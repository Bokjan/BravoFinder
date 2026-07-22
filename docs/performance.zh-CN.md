# 性能测试

> BravoFinder 的性能数据、测试方法与可复现步骤。所有数字都在同一台机器、同一份数据、 同一轮测量下取得，并写明口径。相关代码：`lib/core/graph/`、`lib/io/cache/`、`apps/cli/main.cc`。

## 1. 测试环境

| 项 | 配置 |
|---|---|
| 机器 | AMD EPYC 9K65（单路，本会话分到 16 物理核 / 32 逻辑核） |
| 内存 | 64 GB |
| 系统 | Linux（内核 6.6） |
| 编译器 | gcc 12.3，`x86_64-linux` |
| CMake | 3.26 |
| 构建配置 | `release` 预设（`-O2`），除非另注 |
| 导航数据 | X-Plane 12 native，AIRAC cycle 2601；图 V=270,821 顶点 / E=345,801 边；14838 个机场 CIFP |

> 所有数字都在这台 EPYC 9K65、同一份 cycle 2601 数据、同一轮测量下取得。换机器绝对值会变， 但**相对关系与量级**（启动提速、各优化的相对贡献）应当稳定。

## 2. 启动时间：冷启动 vs 缓存加载

X-Plane 数据的解析 + 建图有固定成本（ARINC 424 解析尤重）。`bf build` 把成果落成 `.bfdb` 缓存，`bf route --db` 反序列化跳过全部解析。

**方法**：`bf route KJFK KLAX` 端到端墙钟时间（含进程启动/退出），各 5 次取稳定值。 release 构建，cycle 2601。

| 路径 | release | 说明 |
|---|---|---|
| 冷启动 `--data navdata`（解析 + 建图） | **~2.27 s** | 每次都重新解析 |
| 缓存加载 `--db nav.bfdb`（on-demand） | **~0.20 s** | 反序列化，跳过解析 |

**约 11× 端到端提速**（2.27s → 0.20s）。`bf build` 本身（一次性，换 AIRAC 周期才跑） 在此机上 ~3.2s，产出一个统一 `.bfdb`（graph + CIFP + detail）60.9 MB。

> debug 预设（含 ASan/UBSan）下冷启动 ~7.6s、缓存 ~1.4s，量级一致。

## 3. 查询耗时与优化分解

启动之外，真正的算法成本在 K-shortest 搜索。这里剥离启动噪声，只测**纯搜索**。

**方法**：进程内微基准（`NavDatabase::OpenCached` 一次，之后循环调用 `FindRoutes`）。
- 工作负载：**10 个真实城市对**（KJFK-KLAX、KSEA-KBOS、KDEN-KSFO、KORD-KDFW、KATL-KLAS、 KMIA-KSEA、KEWR-KSAN、KIAH-KPDX、KPHX-KMSP、KDTW-KSLC），覆盖不同距离/拓扑；
- 每个 pair 先 warmup 一次（填充 on-demand 程序缓存），再计时 **30 轮 × 10 对 = 300 次**， 取平均 ms/search；
- 用 `std::chrono::steady_clock` 只包住 `FindRoutes` 调用。

**优化分解**：四个版本同机、同数据、同工作负载对照（各版算法源码已固化入库，见 `bench/variants/`，用 `bench/decompose.sh` 一键复现，无需 checkout git 历史）：

- **baseline** — 源自 commit `2918c86`（Yen，无 heuristic memoization、无 Lawler）；
- **+memoize** — 源自 commit `ee3afb4`（多目标 heuristic 跨 spur memoization）；
- **+Lawler** — 源自 commit `f7a42c9`（再叠加 Lawler：只从 deviation index 起 spur）；
- **+workspace**（当前）— 每次 spur 复用一份 generation-stamp 搜索工作区（`SearchWorkspace`）， 免掉每次搜索重新分配 + O(V) 初始化 5 个 size-V 数组。

| k | baseline | +memoize | +Lawler | +workspace（当前） | 累计加速 |
|---:|---:|---:|---:|---:|---:|
| 1 | 9.87 | 9.93 | 9.90 | 9.94 | 1.0× |
| 3 | 30.76 | 17.69 | 16.23 | 11.48 | **2.68×** |
| 5 | 51.40 | 24.82 | 20.54 | 11.96 | **4.30×** |
| 10 | 103.92 | 44.42 | 30.21 | 13.20 | **7.87×** |

（单位 ms/search；本机 30 轮 × 10 对 = 300 次/档取平均。绝对值随机器浮动，看同表内相对倍率。）

读这张表：

- **k=1 四版几乎相同**（~9.9ms）。单次搜索里 heuristic 每顶点最多算一次、没有 spur、也只分配 一次工作区——三个优化都不改单次搜索路径，这正是它们**零退化**的证据。
- **memoize** 吃掉 k≥3 的重复 heuristic 计算：goals 集合在整轮 Yen 里恒定，`h(v)` 是常量却被 数百个 spur 重复 O(goals) 扫描。k=10 从 103.9→44.4ms。
- **Lawler** 再砍掉冗余的 spur 搜索本身（只从 deviation index 起 spur）：k=10 从 44.4→30.2ms。
- **workspace** 砍掉每次 spur 重新分配/初始化 5 个 size-V 数组的固定成本（memoize+Lawler 把计算压 下去后，这块固定成本占比反而凸显到 perf 采样的 ~23%）：k=10 从 30.2→13.2ms，几乎减半。
- **收益随 k 增长**：k 越大、spur 越多，三个优化的空间都越大。k=10 累计 **7.87×**。

> 两个优化的原理与正确性论证分别见 [yen-lawler-optimization.zh-CN.md](yen-lawler-optimization.zh-CN.md)（Lawler + memoize）。

## 4. 已止步：profile 指向的固有成本

Lawler 之后再做一轮 profile（gprof，KJFK→KLAX k=10、400 轮、`-pg -O2` 全量编译）。按符号 归类 self time：

| 符号 | self time | 说明 |
|---|---:|---|
| A* 主循环 `RunMultiSearch`（边松弛 / g 更新 / 堆 push，重度内联） | **~93.7%** | Yen 的固有成本 |
| `SeedTable`（每次 spur 前初始化搜索表） | ~0.9% | O(V) 初始化 |
| `Coordinate::DistanceTo` | ~0.6% | memoize 后已压下 |
| heuristic `h(v)`（`MultiGoalHeuristic::operator()`） | ~0.5% | memoize 后已压下 |
| 堆 push（`priority_queue`） | ~0.3% | 已计入主循环 |
| `CostOfPath` | ~0.2% | 噪声级 |
| `std::set` 红黑树（Yen 候选集 + spur 禁集） | ~0.1% | 噪声级 |

（gprof 把内联进主循环的边松弛/g 更新/堆操作都归到 `RunMultiSearch`，且不采样 malloc/系统调用， 故 A* 主循环占比比按调用栈采样的工具更高。**这条工具偏差后来翻了一次案，见下。**）

**结论（及一次翻案）**：两个真热点（重复 heuristic、重复 spur）拿掉后，据此否决了几个直觉性 优化。其中一条**后来被新数据推翻**，记录在此以示「工具决定看得见什么」：

- **✅ 已实现（翻案）：复用搜索数组。** gprof 曾把「每次 spur 分配 + O(V) 初始化 5 个 size-V 数 组」的成本几乎全部内联进 `RunMultiSearch` 的 ~93.7%，独立可见的只剩 `SeedTable` 的 ~0.9%，据此 判为「省不掉躲不掉、收益 ~1%」而否决。后来换 `perf record`（调用栈采样，能把 `std::fill_n` 从主 循环里拆出来）复测 altitude-filtered k=10，发现这块**占 23.04% 自耗时**——gprof 的内联归并掩盖了 它。真相是：memoize+Lawler 把「计算」压下去后，这块「固定分配」的**相对占比**才凸显出来。于是引入 `SearchWorkspace`（栈局部、generation-stamp、O(1) 逻辑清空，不违反「无全局可变状态」也不走 thread_local），k=10 从 30.2→13.2ms。见上表 +workspace 档。
- **❌ 仍不做：Yen 禁集 `std::set` → 排序 vector**——仅 ~0.1%，噪声级收益。
- **❌ 仍不做：`CostOfPath` 线性找边改二分**——~0.2%，且会改 `.bfdb` 布局需 bump format_version。

> 教训：gprof 的「内联归并 + 不采样 malloc」会把分配成本藏进热函数，`perf` 的调用栈采样才拆得开。 单一 profiler 的结论要留意工具偏差——这也是本项目「先 profile 再动手」里 profiler 也要换着看的原因。

再压性能需换算法层（如 Eppstein，或 A* 遍历的预取/SIMD），属大改。

## 5. 内存占用

**方法**：`/usr/bin/time -v` 的 maximum resident set size（进程峰值 RSS，非纯增量）， `bf route KJFK KLAX --db`。

| 模式 | 进程峰值 RSS | 缓存部分增量 |
|---|---:|---:|
| on-demand（默认） | ~101 MB | 程序段仅头 + 目录，+~1.5 MB |
| eager（`--cifp-load eager`） | ~169 MB | 全量程序反序列化，+~67 MB |

- 峰值 RSS 含图（lookup 排序数组 ~4MB + CSR 数组）、进程基线、以及程序段部分。
- 内存紧凑化（2026-07）：per-vertex ident 与 ProcedureLeg.fix 改 12B `FixedIdent`、lookup 哈希表改 排序数组 + 二分、WaypointKind 收窄 U8——on-demand 约省 ~27MB、eager 再省 ~41MB（fix 字段是 eager 的 大头）。详见 `.notes/plans/2026-07-09_memory_compaction.md`。
- CIFP 三字段（2026-07-14，`format_version` 6）：每条 leg 增补 RNP / 转向 / 速度限制（紧凑 u16/u16/char），整文件 +~3.6MB、eager RSS +~1MB；on-demand 不物化全 leg 故基本不变。
- on-demand 适合一次性 CLI 查询（启动省、只加载查到的机场）；eager 适合长驻服务/批量并发 （全量常驻、之后无锁读），见 [thread-safety.zh-CN.md](thread-safety.zh-CN.md)。

## 6. 缓存文件大小

一个统一 `nav_<cycle>.bfdb` 装三段（graph + CIFP + detail），共用一个全局字符串池。实测 cycle 2601 整文件 **60.9 MB**，其中 CIFP 段约 ~42 MB：

| 段 | 约占 | 内容 |
|---|---:|---|
| graph + detail + 全局池 | ~18.9 MB | CSR coords/offsets/edges + in/out 航路成员标志 + idents + airway 名 + MORA + MSA + 导航台细节 + 等待航线 + 全局池（~1.5MB） |
| cifp | ~42 MB | 14838 机场分段程序（结构化后远小于 ~105MB 原始 CIFP 文本） |

全局池三段共用、去重后 ~1.5MB（三段各自局部池之和 ~8.7MB → −83%），是整文件比旧三文件分离 省 ~8MB 的主因。格式与取舍见 [binary-cache.zh-CN.md](binary-cache.zh-CN.md)。

## 7. 如何复现

启动与端到端（用发布的 CLI）：

```bash
cmake --preset release && cmake --build --preset release
bf build navdata -o /tmp/nav.bfdb              # 一次性建缓存，计时见 bf build

# 冷启动 vs 缓存，各跑几次
for i in 1 2 3 4 5; do /usr/bin/time -p bf route KJFK KLAX --data navdata >/dev/null; done
for i in 1 2 3 4 5; do /usr/bin/time -p bf route KJFK KLAX --db /tmp/nav.bfdb >/dev/null; done

# 内存（Linux 用 /usr/bin/time -v 的 Maximum resident set size；macOS 用 -l）
/usr/bin/time -v bf route KJFK KLAX --db /tmp/nav.bfdb >/dev/null
/usr/bin/time -v bf route KJFK KLAX --db /tmp/nav.bfdb --cifp-load eager >/dev/null
```

纯搜索的分解用进程内微基准 `bench/route_bench.cc`（`OpenCached` 一次 + 循环 `FindRoutes` 10 城市对 × 30 轮，`steady_clock` 只包 `FindRoutes`）。四版对照的算法源码 （`lib/core/graph/astar.*` + `yen_kshortest.*`）已固化在 `bench/variants/{baseline,memoize,lawler,workspace}/`， `bench/decompose.sh` 用 `-DBRAVOFINDER_BENCH_VARIANT` 分别编译对照二进制，无需 checkout git 历史、也不污染主工作区。profile 用 gprof：`-pg -O2` 全量编译微基准（把 `lib/core/`+`lib/io/` 的 。cc 与基准一起编，需 `-I build/<preset>/core` 找生成的 `version.h`），跑一轮后 `gprof <bin> gmon.out`。基准工具默认不入构建（`BRAVOFINDER_BUILD_BENCH=OFF`），复现步骤见 `bench/README.md`。

## 8. 小结

- **启动**：缓存把冷启动 2.27s 降到 0.20s，**~11×**（换 AIRAC 才需重建，3.2s 一次性）。
- **查询**：memoize + Lawler + workspace 叠加，k=10 从 103.9ms 降到 13.2ms，**7.87×**；k=1 零退化；收益随 k 增长。
- **止步有据（含一次翻案）**：gprof 曾因内联归并把「复用搜索数组」判为 ~1% 而否决，`perf` 调用栈采样揭示其达 23%，遂实现（workspace 档）；其余两个微优化仍不做。
- **内存/文件**：on-demand ~101MB / eager ~169MB 峰值 RSS（2026-07 紧凑化后）；统一 `.bfdb` 60.9MB（CIFP 段 ~42MB）。
