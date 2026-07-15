# HTTP 查询服务（bf-http）

> 面向读者的架构说明。CLI 用法在仓库根 [README.md](../README.md) 的「HTTP server」段；
> 本文讲**内部怎么工作、为什么这么设计**。

## 定位：服务化，不做 in-process binding

`bf-http` 是一个**内网航路查询服务**：上层业务（Go 网关）通过 HTTP+JSON 调用它，而不是把
C++ 库以 cgo/pybind 方式嵌进业务进程。理由：

- 一次路由查询本身是 **~10ms(k=1) ~ 13–30ms(k=10)** 的纯计算（见 [性能测试](performance.zh-CN.md)），
  相比之下 JSON 序列化（µs 级）、localhost 往返（亚 ms）都低 1–3 个数量级——"为省编组开销选
  in-process"不成立。
- 数据模型天然"加载一次、服务多次"：`OpenCached(kEager)` 常驻后无锁并发（[线程安全契约 B](thread-safety.zh-CN.md)）。
- 进程隔离：C++ 崩溃不连坐 Go 网关，Go 保持 `CGO_ENABLED=0`。
- 消费侧要强类型对象时，Go 的 `encoding/json` 反序列化即得——比 cgo 手写 C-struct 镜像省事得多。

## 复用：中性 query_core / `bf::service`

查询逻辑不属于任何一种传输。九个 handler（`find_routes` / `parse_route` / 各批量 lookup）+
多周期 `NavDatabaseRegistry` 都在 **`apps/query_core`（命名空间 `bf::service`）**里，MCP 与 HTTP
**平级依赖**它，没有 `http → mcp` 的别扭依赖。

handler 返回 `HandlerResult{body, status}`（HTTP 风格状态码）。两种传输各取所需：MCP 只看
`is_error = (status >= 400)`；HTTP 直接用 status。新增一个查询能力 = 在 `bf::service` 加一个
handler，两端自动受益。

## 传输：手搓 libuv + llhttp

- **libuv**：事件循环 + 内置线程池，统一 API 下自动走 epoll(Linux)/kqueue(macOS)/IOCP(Windows)，
  天然跨平台，原生支持 MSVC。
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

**铁律：10–30ms 的路由计算绝不在 loop 线程上跑。** 用 `uv_queue_work(work, after_work)` offload
到线程池；worker 只碰 `registry`/`handler`/`args`/`result`，**绝不碰 libuv handle**（handle 非线程
安全）。`registry` 与各 `NavDatabase` 单实例跨所有线程共享，只读、并发安全（契约 B）。起步单
loop 即可扛高连接数（重计算已 offload）。

### 连接生命周期与存活守卫（头号并发陷阱）

客户端可能在 worker 计算途中断开。做法：

- `Connection` 用 `std::shared_ptr` 持有；它保留一个 `self_` 强引用，跨 libuv 回调存活。
- 每个在途 work item **额外持一个强引用**，所以即便客户端中途断开、loop 线程已 `uv_close` 了它的
  handle，`Connection` 对象与其 handle 内存仍存活到 work 完成。
- `after_work` 回到 loop 线程后先查 `IsAlive()`：连接已关就**丢弃响应**，不写。
- `self_` 只在两个 handle（tcp + timer）都关完后释放，对象随最后一个在途引用消失而析构。

集成测试专门覆盖"计算途中断开不崩"（`tests/integration/http_server_test.cc`），并跑 tsan。

### 手搓 HTTP 的安全硬化

llhttp 只解析，HTTP/1.1 的语义与安全都在 `conn.cc` 里自己接（缺一不可）：

- **keep-alive**：依 `llhttp_should_keep_alive` 复用连接；复用前**完整重置** parser 与请求缓冲，
  防上一请求残留串到下一请求。
- **响应帧**：手写状态行 + `Content-Type`/`Content-Length`/`Connection`/`Date`；只发 `Content-Length`。
- **上限**：header 总长/条数上限（→ 431）、body 上限（`--max-body` → 413），防内存耗尽。
- **超时**：单个 idle 定时器覆盖 header 读取 / body 读取 / 空闲 keep-alive（`--io-timeout`），防
  slowloris 慢连接占用。
- **chunked 拒绝**：内网 JSON API 不需要 `Transfer-Encoding: chunked` → **显式拒绝**（400 + 关连接），
  安全地拒而非误解析，防请求走私。
- **单请求在飞**：不做 pipelining——一个请求分发后忽略后续输入字节，直到响应写完（但仍响应断开）。
- **异常兜底**：worker 里 handler 理论只走 `Result`，仍 try/catch 兜住意外异常 → 500，绝不让异常
  穿过线程边界。

## 端点与错误模型

所有查询端点 **POST + JSON body**（批量 lookup 入参 `{"ids":[...]}`，单查=一元素数组）；`?cycle=2601`
选周期，缺省取最新。统一错误负载 `{"error":"..."}`（rapidjson 自动转义用户可控串）。

| HTTP 状态 | 场景 |
|---|---|
| **400** | JSON 解析失败 / 缺必填字段 / 参数非法（k<1、min>max）/ 未知无效 `?cycle=` |
| **404** | 批量 lookup 全未命中 / procedure-legs 无匹配 / 未注册路径 |
| **422** | `FindRoutes`/`ParseRoute` 语义失败（未知端点、无航路、坏 token）——语法合法但无解 |
| **413** | 请求体超 `--max-body` |
| **400** | `Transfer-Encoding: chunked` 请求体（显式拒绝） |
| **500** | worker 未捕获异常 |

区分"你传错了"(400) 与"你没传错但无解"(422)，是错误模型的核心。

探针：`/healthz` 恒 200（进程存活）；`/readyz` 最新周期可打开才 200，否则 503（且开库可能有磁盘
I/O，故 readyz 也走 offload）。

## 合规

libuv、llhttp 均为 **MIT**，编译进 `bf-http` 二进制，义务=保留许可证文本——见
[THIRD_PARTY_LICENSES.md](../THIRD_PARTY_LICENSES.md)，发版产物一并携带。
