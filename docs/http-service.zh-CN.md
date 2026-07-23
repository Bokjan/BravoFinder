# HTTP 查询服务（bf-http）

> 面向读者的架构说明。CLI 用法在仓库根 [README.md](../README.md) 的「HTTP server」段； 本文讲**内部怎么工作、为什么这么设计**。

## 定位：服务化，不做 in-process binding

`bf-http` 是一个**内网航路查询服务**：上层业务（Go 网关）通过 HTTP+JSON 调用它，而不是把 C++ 库以 cgo/pybind 方式嵌进业务进程。理由：

- 一次路由查询本身是 **~10ms(k=1) ~ 13–30ms(k=10)** 的纯计算（见 [性能测试](performance.zh-CN.md)）， 相比之下 JSON 序列化（µs 级）、localhost 往返（亚 ms）都低 1–3 个数量级——"为省编组开销选 in-process"不成立。
- 数据模型天然"加载一次、服务多次"：`OpenCached(kEager)` 常驻后无锁并发（[线程安全契约 B](thread-safety.zh-CN.md)）。
- 进程隔离：C++ 崩溃不连坐 Go 网关，Go 保持 `CGO_ENABLED=0`。
- 消费侧要强类型对象时，Go 的 `encoding/json` 反序列化即得——比 cgo 手写 C-struct 镜像省事得多。

## 复用：中性 service 层 / `bf::service`

查询逻辑不属于任何一种传输。九个 handler（`find_routes` / `parse_route` / 各批量 lookup）+ 多周期 `NavDatabaseRegistry` 都在 **顶层 `service/`（命名空间 `bf::service`，target `bf_service_lib`）**里，MCP 与 HTTP **平级依赖**它，没有 `http → mcp` 的别扭依赖。它是 app 层库（带 rapidjson），与 `lib/` 引擎分开，保住引擎无 JSON/网络依赖。

handler 返回 `HandlerResult{body, status}`（HTTP 风格状态码）。两种传输各取所需：MCP 只看 `is_error = (status >= 400)`；HTTP 直接用 status。新增一个查询能力 = 在 `bf::service` 加一个 handler，两端自动受益。

## 传输：手搓 libuv + llhttp（共享核心 `http_server/`）

传输本身也不属于任何一种消费者。连接状态机、TCP listener、线程池 offload、`RequestHandler` 接口与传输本地的 `WorkResult`，都在 **顶层 `http_server/`（命名空间 `bf::http_server`，target `bf_http_server`）**：REST 服务（`apps/http`）与 MCP-over-HTTP 传输（`apps/mcp` http 模式）**共用同一份**。它是 **JSON/查询中性**的——不依赖 `bf_service_lib`（正如 `lib/` 不依赖网络），消费者实现 `RequestHandler` 赋予请求含义。（其头文件名是 `transport.h` 而非 `dispatcher.h`：后者是 MCP 的 JSON-RPC dispatcher 专名，两目录在编译 `bf_mcp_lib` 时都在 include 路径上，同名会歧义。）

- **libuv**：事件循环 + 内置线程池，统一 API 下自动走 epoll(Linux)/kqueue(macOS)/IOCP(Windows)， 天然跨平台，原生支持 MSVC。
- **llhttp**：Node 的 HTTP/1.1 解析器，只做**解析**（喂字节 → 回调）。

### 线程拓扑与异步铁律

```
libuv loop 线程(epoll)  ── accept / read / llhttp 解析 / 写回，全非阻塞
        │  uv_queue_work：派发 {args, cycle, 连接强引用}
        ▼
libuv 内置线程池  ── registry.Get(cycle) → handler (10–30ms CPU)
        │  after_work 回调（自动回到 loop 线程）
        ▼
loop 线程：连接仍存活则写响应，否则丢弃结果
```

**铁律：10–30ms 的路由计算绝不在 loop 线程上跑。** 用 `uv_queue_work(work, after_work)` offload 到线程池；worker 只碰 `registry`/`handler`/`args`/`result`，**绝不碰 libuv handle**（handle 非线程 安全）。`registry` 与各 `NavDatabase` 单实例跨所有线程共享，只读、并发安全（契约 B）。起步单 loop 即可扛高连接数（重计算已 offload）。

### 连接生命周期与存活守卫（头号并发陷阱）

客户端可能在 worker 计算途中断开。做法：

- `Connection` 用 `std::shared_ptr` 持有；它保留一个 `self_` 强引用，跨 libuv 回调存活。
- 每个在途 work item **额外持一个强引用**，所以即便客户端中途断开、loop 线程已 `uv_close` 了它的 handle，`Connection` 对象与其 handle 内存仍存活到 work 完成。
- `after_work` 回到 loop 线程后先查 `IsAlive()`：连接已关就**丢弃响应**，不写。
- `self_` 只在两个 handle（tcp + timer）都关完后释放，对象随最后一个在途引用消失而析构。

集成测试专门覆盖"计算途中断开不崩"（`tests/integration/http_test.cc`），并跑 tsan。

### 手搓 HTTP 的安全硬化

llhttp 只解析，HTTP/1.1 的语义与安全都在 `conn.cc` 里自己接（缺一不可）：

- **keep-alive**：依 `llhttp_should_keep_alive` 复用连接；复用前**完整重置** parser 与请求缓冲， 防上一请求残留串到下一请求。
- **响应帧**：手写状态行 + `Content-Type`/`Content-Length`/`Connection`/`Date`；只发 `Content-Length`。
- **上限**：header 总长/条数上限（→ 431）、body 上限（`--max-body` → 413），防内存耗尽。
- **超时**：单个 idle 定时器覆盖 header 读取 / body 读取 / 空闲 keep-alive（`--io-timeout`），防 slowloris 慢连接占用。
- **chunked 拒绝**：内网 JSON API 不需要 `Transfer-Encoding: chunked` → **显式拒绝**（400 + 关连接）， 安全地拒而非误解析，防请求走私。
- **单请求在飞**：不做 pipelining——一个请求分发后忽略后续输入字节，直到响应写完（但仍响应断开）。
- **异常兜底**：worker 里 handler 理论只走 `Result`，仍 try/catch 兜住意外异常 → 500，绝不让异常 穿过线程边界。

## 端点与错误模型

所有查询端点 **POST + JSON body**（批量 lookup 入参 `{"ids":[...]}`，单查=一元素数组）；`?cycle=2601` 选周期，缺省取最新。统一错误负载 `{"error":"..."}`（rapidjson 自动转义用户可控串）。

| HTTP 状态 | 场景 |
|---|---|
| **400** | JSON 解析失败 / 缺必填字段 / 参数非法（k<1、min>max）/ 未知无效 `?cycle=` |
| **404** | 批量 lookup 全未命中 / procedure-legs 无匹配 / 未注册路径 |
| **422** | `FindRoutes`/`ParseRoute` 语义失败（未知端点、无航路、坏 token）——语法合法但无解 |
| **413** | 请求体超 `--max-body` |
| **400** | `Transfer-Encoding: chunked` 请求体（显式拒绝） |
| **500** | worker 未捕获异常 |

区分"你传错了"(400) 与"你没传错但无解"(422)，是错误模型的核心。

探针：`/healthz` 恒 200（进程存活）；`/readyz` 最新周期可打开才 200，否则 503（且开库可能有磁盘 I/O，故 readyz 也走 offload）。

## MCP-over-HTTP 传输（`bf-mcp --transport http`）

`bf-mcp` 是同一套查询能力的另一个消费者。它默认 **stdio**（本地 MCP client spawn，向后兼容），也可切到 **HTTP**：`bf-mcp --transport http --port 8080`，供远程 / 多客户端 MCP client 接入。REST（`bf-http`）不受影响、独立二进制——`BravoFinderWeb` 的 Go 网关按 path+method 字节透传 REST、从不解析 body，砍掉 REST 换纯 MCP 会把它退化成 JSON-RPC↔REST 翻译桥，故 REST 保留。

- **协议核心复用**：两种 MCP 传输共用 transport-neutral 的 `Dispatcher`（`apps/mcp/dispatcher.{h,cc}`，method 分发 + 协议版本协商 + 工具表 + **batch**）。`Dispatch` 同时认单个对象与 batch（JSON 数组）：batch 逐元素分发、收集非 notification 项为数组，全 notification 则无响应；非对象元素回一个 `id:null` 的 `-32600`（不静默丢弃），空 batch 回单个 `-32600` error 对象（非数组，per JSON-RPC 2.0）。正因为 batch 在 `Dispatcher` 里，**stdio 也支持 batch**——一行 batch 请求进、一行数组响应出。stdio 内联 `Dispatch`；HTTP 把整个 `Dispatch` offload 到线程池（`Dispatch` 是 `const`、只读 registry/tools，多 worker 并发安全）。`stdio_runner` 与 `mcp_http` 只是两层薄壳。错误码（`-32700`/`-32600`/`-32601`/`-32602`，JSON-RPC 2.0 §5.1 规范定义）收口在 `apps/mcp/jsonrpc.h` 的 `constexpr`，不散落字面量。
- **传输核心复用**：HTTP 模式建在 `bf_http_server` 上，与 REST 同源——10–30ms 的 `tools/call` 计算走同一套 `uv_queue_work` offload + 连接存活守卫。offload 的工作单元是 **可拷贝** 的 `std::function<WorkResult()>`（解析后的请求用 `shared_ptr<Document>` 持有，move-capture 的 Document 不可拷贝、进不了 `std::function`）。
- **Streamable HTTP（2025-03-26）单端点 `/mcp`**：
  - `POST /mcp`：body 是 JSON-RPC 请求（单个或批量，批量语义见上节 `Dispatcher`）。纯 notification（无 id）→ `202 Accepted` 空体；否则 offload dispatch，完成后按 `Accept` 头决定响应帧——含 `text/event-stream` → `200` + 单个 SSE 事件（`data: <json-rpc>\n\n`），否则 `200` `application/json`。空 batch 例外：它是合法 JSON 但非法 JSON-RPC 消息，回 `200` + 单个 `-32600` error 对象（不是 400 framing 错误，也不是数组）。
  - `GET /mcp`：开一条 SSE 长连接（`text/event-stream`，close-delimited 开放流），发一个 `: keepalive` 注释即占位。当前工具无 progress、无 server-push，此流零消费者，仅为将来通知预留 + 合成测试覆盖。
  - `DELETE /mcp`：无状态 → `200` ack。
- **Session（无状态带 id）**：`initialize`（单个请求或 batch 内含一个）响应回一个随机 `Mcp-Session-Id` 头并协商到 2025-03-26；后续请求接受任意 session id 但**不跟踪**（Dispatcher 无 per-session 状态，每个请求独立）。
- **SSE 写出能力**（加在 `http_server/conn.cc`）：`WriteResponse` 支持自定义 `content_type` + 额外 header（POST→SSE 单事件用 `Content-Length` 缓冲）；`BeginStream`/`WriteEvent` 是 close-delimited 开放流（无 `Content-Length`、`Connection: keep-alive`），靠一个 `streaming_` 标志让写完成回调既不复位下一请求也不关连接，直到客户端断开 / idle 超时。**SSE 写出全在 loop 线程**，worker 只算不写。改这块并发面必过 `ctest --preset tsan`（契约 B）。

端到端覆盖在 `tests/integration/mcp_http_test.cc`（真 loopback，用最小手造缓存，不依赖真实 navdata、不 SKIP）。

## 合规

libuv、llhttp 均为 **MIT**，编译进 `bf-http` 二进制，义务=保留许可证文本——见 [THIRD_PARTY_LICENSES.md](../THIRD_PARTY_LICENSES.md)，发版产物一并携带。
