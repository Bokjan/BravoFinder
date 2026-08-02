# BravoFinder 文档

> 面向使用者的介绍与 CLI 用法在仓库根的 [README.md](../README.md)。这里是**面向读者的 技术文档**：讲清 BravoFinder 有什么特色、内部怎么工作。文章用中文（`*.zh-CN.md`）。

## 从这里开始

不熟悉图算法？先读这篇入门，其余专题都建立在它的概念上：

- **[航路查找的基础算法](routing-basics.zh-CN.md)** — 图 / 最短路 / A\*（大圆距离启发）/ Yen K-shortest，从零讲清 BravoFinder 怎么找出一条航路。

## 设计特色

按「是什么 → 怎么建模 → 怎么快 → 怎么并发 → 实测多快」的顺序：

- **[合规航路引擎](compliant-routing.zh-CN.md)** — 为什么不是地理最短：航路方向性、高低空 分层、高度带、MORA、可插拔约束框架、Yen 多候选择优。
- **[地形安全：MORA 网格与 MSA 扇区](terrain-safety.zh-CN.md)** — 两种最低安全高度的建模： MORA 1° 全球稠密网格 vs MSA 按机场的稀疏扇区，为何用两套结构、怎么参与算路。
- **[程序建模与航路网衔接](procedure-modeling.zh-CN.md)** — ARINC 424 / CIFP 全 23 种 path terminator、机场靠真实 SID/STAR 接入、衔接 fix 选点、多源 K-shortest、雷达引导的语义诚实。
- **[航路串压缩](route-string.zh-CN.md)** — 输出符合 ICAO filed-flight-plan 的压缩航路串： 并线航路（`V28-Y28`）的累积交集折叠，避开会造出假转接点的字符串相等陷阱。
- **[Yen 的 Lawler 优化](yen-lawler-optimization.zh-CN.md)** — heuristic 记忆化 + Lawler， 把 K-shortest 提速约 2.5× 且结果一字不差；含非标准多源变体的正确性论证与差分对拍验证。
- **[跨平台二进制缓存](binary-cache.zh-CN.md)** — 统一 `.bfdb`：为何显式定宽 小端而非 mmap、全局字符串池、容器/codec 分层、on-demand / eager 加载、损坏优雅报错。
- **[线程安全契约](thread-safety.zh-CN.md)** — 一个只读实例多线程并发查询：双检锁、指针 跨 rehash 稳定、eager 冻结无锁读、tsan 验证。
- **[领域建模与内存设计](domain-design.zh-CN.md)** — 值类型、自研 `Result<T,E>`、无 static/ 无裸 new 的设计宪法、数据驱动的紧凑内存表示（`SmallVec`→`FixedIdent`）；v2→v3 重写动机的直接体现。
- **[HTTP 查询服务](http-service.zh-CN.md)** — `bf-http` REST 与 `bf-mcp --transport http`（Streamable HTTP）：为何服务化优于 in-process binding、复用中性 `bf::service` 与共享 `libs/http_server/` 传输核心、libuv + llhttp 手搓、offload 线程模型与连接存活守卫、HTTP 安全硬化、状态码错误模型、MCP `Dispatcher` 与 batch / session 语义。
- **[性能测试](performance.zh-CN.md)** — 测试机配置、方法、启动 ~11×、优化分解表、内存与 缓存大小、可复现步骤。

## 贡献

- **[CONTRIBUTING.md](CONTRIBUTING.md)** — 提交规范（Conventional Commits）、语言规则、 代码风格、版本纪律、测试要求。
