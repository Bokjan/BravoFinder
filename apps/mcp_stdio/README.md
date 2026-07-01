# bf-mcp-stdio

BravoFinder 的本地 MCP server，把 `bf route` 和 `bf query` 的能力以 MCP tools 的
形式通过 stdio 暴露给 LLM 客户端。本文件供 **agent / 配置助手** 阅读：前半部分讲
怎么把 server 跑起来并接进 MCP 客户端，后半部分是 tool 能力速查。

## 这是什么

- 传输方式：**stdio**（本地进程，客户端拉起子进程经 stdin/stdout 通信）。
- 协议：MCP over JSON-RPC 2.0，零依赖手写（不引第三方 MCP SDK）。
- 能力：5 个 tools，与 CLI 的 `route` / `query` 子命令一一对应。
- 数据：server 启动时加载一个预建的 `.bfdb` 缓存（只读），不解析原始数据、不写文件。

## 构建

依赖项目自身的 `bf` 库和 RapidJSON，纯 CMake + FetchContent，无额外依赖。

```bash
cmake --preset release && cmake --build --preset release
# 或 debug：
cmake --preset debug   && cmake --build --preset debug
```

产物路径（target 名 `bf_mcp_stdio`，对外名 `bf-mcp-stdio`）：

```
build/release/apps/mcp_stdio/bf-mcp-stdio
build/debug/apps/mcp_stdio/bf-mcp-stdio
```

## 启动与缓存定位

server 启动时调用 `NavDatabase::OpenCached` 加载图缓存，**一次性**。缓存缺失或
损坏则进程直接以非 0 退出并向 stderr 打印原因（早失败，MCP 客户端会报启动失败）。

缓存路径按以下优先级确定：

1. 命令行 `--db <path>` 显式指定图缓存；否则用 `<data_dir>/nav.bfdb`。
2. `data_dir`：命令行 `--data <dir>` 指定；否则读环境变量 `BRAVOFINDER_NAVDATA`；
   再否则默认 `navdata/`。
3. 程序缓存：`--cifp-db <path>` 指定；否则自动找 `--db` 同目录下的
   `<stem>_cifp.bfdb`（若存在）；再否则回退到从 `data_dir` 里的 CIFP 文件按需解析。

```bash
# 最简：navdata/ 下要有 nav.bfdb（和可选的 nav_cifp.bfdb），由 BRAVOFINDER_NAVDATA 定位
BRAVOFINDER_NAVDATA=navdata bf-mcp-stdio

# 显式指定两个缓存
bf-mcp-stdio --db navdata/nav.bfdb --cifp-db navdata/nav_cifp.bfdb

# 只给图缓存，程序缓存由同目录兄弟文件自动发现
bf-mcp-stdio --db /path/to/nav.bfdb

# 指定数据目录（用于按需解析程序，若没给 --cifp-db）
bf-mcp-stdio --data /path/to/xplane/data
```

> 缓存由 `bf build` 生成（见仓库根 README 的 `bf build` 用法）。`bf-mcp-stdio`
> 本身**不**暴露 `build`——建缓存是 CLI / 部署脚本的职责。

## 接进 MCP 客户端

在客户端的 MCP server 配置里，把 `command` 指向编译出的二进制，`args` 传启动参数。
示例（Claude Desktop / Cursor / 类似客户端）：

```json
{
  "mcpServers": {
    "bravofinder": {
      "command": "/abs/path/to/bf-mcp-stdio",
      "args": ["--db", "navdata/nav.bfdb"]
    }
  }
}
```

带环境变量的写法（多数客户端支持 `env` 字段）：

```json
{
  "mcpServers": {
    "bravofinder": {
      "command": "/abs/path/to/bf-mcp-stdio",
      "args": ["--data", "/path/to/xplane/data"],
      "env": { "BRAVOFINDER_NAVDATA": "/path/to/navdata" }
    }
  }
}
```

## Tool 能力速查

所有 tool 的请求参数都放在 MCP `tools/call` 的 `arguments` 对象里。返回统一包装为
`{ content: [{ type: "text", text: <json> }], isError: <bool> }`，其中 `text` 是一段
JSON（对象或数组），直接解析即可。

| Tool | 必填参数 | 可选参数 | 返回 |
|------|---------|---------|------|
| `find_routes` | `departure`, `arrival` | `cruise_fl`, `level`, `k`, `departure_runway`, `arrival_runway`, `departure_sid`, `arrival_star` | 候选航路数组（见下） |
| `lookup_waypoints` | `ids` (string[]) | — | 与 `ids` 平行的数组，元素为 waypoint 对象或 `null` |
| `lookup_airports` | `ids` (string[]) | — | 同上，airport 对象或 `null` |
| `lookup_procedures` | `ids` (string[]) | — | 同上，procedures 对象或 `null` |
| `lookup_airways` | `ids` (string[]) | — | 同上，airway 对象或 `null` |

### `find_routes`

参数语义：

- `departure` / `arrival`：机场 ICAO 或 waypoint ident（大小写不敏感）。
- `cruise_fl`：巡航飞行高度层（百英尺），如 `350` 表示 FL350；设置后启用高度带 / MORA 约束过滤。
- `level`：`none`（默认）| `low`（优先 Victor 低空航路）| `high`（优先 Jet 高空航路）。
- `k`：返回的候选航路数（Yen K-shortest），默认 1，需 ≥ 1。
- `departure_runway` / `arrival_runway`：限制所用 SID / STAR 的跑道，如 `RW31L`；空 = 任意。
- `departure_sid` / `arrival_star`：指定 SID / STAR 名称（如 `DEEZZ5` 或 `DEEZZ5.TOWIN` 钉死过渡段）；空 = 自动选。

返回数组每个元素字段：`route`（ICAO 申报式航路串）、`total_distance_nm`、`sid`、
`dep_runway`、`sid_options`、`star`、`arr_runway`、`star_options`、
`dep_connection` / `arr_connection`（`procedure` / `radar_vectors` 等）、`legs`
（每段的 `from` / `to` / `via` / `distance_nm`，并发航段额外带 `concurrent_airways`）。

### lookup 系列

`ids` 为字符串数组，返回数组与 `ids` **顺序平行**，查不到的对应位置为 `null`。
`lookup_procedures` 的 `ids` 是机场 ICAO；其余按 ident / 设计器名查。

## 错误语义

- tool 调用成功：`isError: false`。
- `find_routes` 算不出航路（端点未知 / 无可行航路）：`isError: true`，`text` 为
  `{"error":"..."}`。
- 某 lookup 的 `ids` **全部**缺失：`isError: true`；**部分**命中：`isError: false`
  （缺失项在数组里是 `null`）。
- 未知 tool 名：`isError: true`。
- 协议层错误（如 `tools/call` 缺 `params` 对象）：返回 JSON-RPC `error`，非 tool 结果。

## 并发与生命周期

`NavDatabase` 在 `OpenCached` 后只读，可多线程并发查询（契约 B）。客户端持有一个
长驻 server 实例反复调用 `tools/call` 即可，无需每次重启。
