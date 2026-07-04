# 性能测试

> BravoFinder 的性能数据、测试方法与可复现步骤。所有数字都在同一台机器、同一份数据、
> 同一轮测量下取得，并写明口径。相关代码：`core/graph/`、`io/cache/`、`apps/cli/main.cc`。

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

> 所有数字都在这台 EPYC 9K65、同一份 cycle 2601 数据、同一轮测量下取得。换机器绝对值会变，
> 但**相对关系与量级**（启动提速、各优化的相对贡献）应当稳定。

## 2. 启动时间：冷启动 vs 缓存加载

X-Plane 数据的解析 + 建图有固定成本（ARINC 424 解析尤重）。`bf build` 把成果落成
`.bfdb` 缓存，`bf route --db` 反序列化跳过全部解析。

**方法**：`bf route KJFK KLAX` 端到端墙钟时间（含进程启动/退出），各 5 次取稳定值。
release 构建，cycle 2601。

| 路径 | release | 说明 |
|---|---|---|
| 冷启动 `--data navdata`（解析 + 建图） | **~2.27 s** | 每次都重新解析 |
| 缓存加载 `--db nav.bfdb`（on-demand） | **~0.20 s** | 反序列化，跳过解析 |

**约 11× 端到端提速**（2.27s → 0.20s）。`bf build` 本身（一次性，换 AIRAC 周期才跑）
在此机上 ~3.2s，产出一个统一 `.bfdb`（graph + CIFP + detail）57.3 MB。

> debug 预设（含 ASan/UBSan）下冷启动 ~7.6s、缓存 ~1.4s，量级一致。

## 3. 查询耗时与优化分解

启动之外，真正的算法成本在 K-shortest 搜索。这里剥离启动噪声，只测**纯搜索**。

**方法**：进程内微基准（`NavDatabase::OpenCached` 一次，之后循环调用 `FindRoutes`）。
- 工作负载：**10 个真实城市对**（KJFK-KLAX、KSEA-KBOS、KDEN-KSFO、KORD-KDFW、KATL-KLAS、
  KMIA-KSEA、KEWR-KSAN、KIAH-KPDX、KPHX-KMSP、KDTW-KSLC），覆盖不同距离/拓扑；
- 每个 pair 先 warmup 一次（填充 on-demand 程序缓存），再计时 **30 轮 × 10 对 = 300 次**，
  取平均 ms/search；
- 用 `std::chrono::steady_clock` 只包住 `FindRoutes` 调用。

**优化分解**：三个版本同机、同数据、同工作负载对照（用 git 历史文件编译对照二进制）：

- **baseline** — commit `2918c86`（Yen，无 heuristic memoization、无 Lawler）；
- **+memoize** — commit `ee3afb4`（多目标 heuristic 跨 spur memoization）；
- **+Lawler**（当前）— commit `f7a42c9`（再叠加 Lawler 优化）。

| k | baseline | +memoize | +Lawler（当前） | 累计加速 |
|---:|---:|---:|---:|---:|
| 1 | 8.10 | 8.08 | 8.07 | 1.0× |
| 3 | 25.67 | 15.56 | 14.34 | **1.79×** |
| 5 | 43.08 | 22.78 | 18.77 | **2.29×** |
| 10 | 87.58 | 42.16 | 28.53 | **3.07×** |

（单位 ms/search。）

读这张表：

- **k=1 三版几乎相同**（~8.1ms）。单次搜索里 heuristic 每顶点最多算一次、也没有 spur，
  两个优化都不改单次搜索路径——这正是它们**零退化**的证据。
- **memoize** 主要吃掉 k≥3 的重复 heuristic 计算：goals 集合在整轮 Yen 里恒定，`h(v)` 是常量
  却被数百个 spur 重复 O(goals) 扫描。k=10 从 87.6→42.2ms。
- **Lawler** 再砍掉冗余的 spur 搜索本身（只从 deviation index 起 spur）：k=10 从 42.2→28.5ms。
- **收益随 k 增长**：k 越大、spur 越多、重复越多，两个优化的空间越大。k=10 累计 **3.07×**。

> 两个优化的原理与正确性论证分别见
> [yen-lawler-optimization.zh-CN.md](yen-lawler-optimization.zh-CN.md)（Lawler + memoize）。

## 4. 已止步：profile 指向的固有成本

Lawler 之后再做一轮 profile（gprof，KJFK→KLAX k=10、400 轮、`-pg -O2` 全量编译）。按符号
归类 self time：

| 符号 | self time | 说明 |
|---|---:|---|
| A* 主循环 `RunMultiSearch`（边松弛 / g 更新 / 堆 push，重度内联） | **~93.7%** | Yen 的固有成本 |
| `SeedTable`（每次 spur 前初始化搜索表） | ~0.9% | O(V) 初始化 |
| `Coordinate::DistanceTo` | ~0.6% | memoize 后已压下 |
| heuristic `h(v)`（`MultiGoalHeuristic::operator()`） | ~0.5% | memoize 后已压下 |
| 堆 push（`priority_queue`） | ~0.3% | 已计入主循环 |
| `CostOfPath` | ~0.2% | 噪声级 |
| `std::set` 红黑树（Yen 候选集 + spur 禁集） | ~0.1% | 噪声级 |

（gprof 把内联进主循环的边松弛/g 更新/堆操作都归到 `RunMultiSearch`，且不采样 malloc/系统调用，
故 A* 主循环占比比按调用栈采样的工具更高；量级结论一致：**A* 遍历本身是绝对热点**。）

**结论**：两个真热点（重复 heuristic、重复 spur）已被拿掉，剩下几乎全是 A* 遍历的固有成本。
据此**否决了几个直觉性优化**（数据说话）：

- **栈局部 buffer / thread_local 复用搜索数组**——分配/初始化（`SeedTable` 类）合计仅 ~1%，且
  省不掉躲不掉的 O(V) 初始化；thread_local 还违反"无全局可变状态"，线程池下每线程 ~5.4MB 永久
  常驻。不做。
- **Yen 禁集 `std::set` → 排序 vector**——仅 ~0.1%，噪声级收益。不做。
- **`CostOfPath` 线性找边改二分**——profile 里 ~0.2%，且会改 `.bfdb` 布局需 bump
  format_version。不做。

再压性能需换算法层（如 Eppstein，或 A* 遍历的预取/SIMD），属大改，不在 v3.0.0 范围。

## 5. 内存占用

**方法**：`/usr/bin/time -v` 的 maximum resident set size（进程峰值 RSS，非纯增量），
`bf route KJFK KLAX --db`。

| 模式 | 进程峰值 RSS | 缓存部分增量 |
|---|---:|---:|
| on-demand（默认） | ~128 MB | 程序段仅头 + 目录，+~1.5 MB |
| eager（`--cifp-load eager`） | ~234 MB | 全量程序反序列化，+~100 MB |

- 峰值 RSS 含图（~30MB lookup map + CSR 数组）、进程基线、以及程序段部分。
- on-demand 适合一次性 CLI 查询（启动省、只加载查到的机场）；eager 适合长驻服务/批量并发
  （全量常驻、之后无锁读），见 [thread-safety.zh-CN.md](thread-safety.zh-CN.md)。

## 6. 缓存文件大小

一个统一 `nav_<cycle>.bfdb` 装三段（graph + CIFP + detail），共用一个全局字符串池。实测
cycle 2601 整文件 **57.3 MB**；`--without-cifp`（仅 graph + detail + 池）**18.9 MB**，反推
CIFP 段 **~38 MB**：

| 段 | 约占 | 内容 |
|---|---:|---|
| graph + detail + 全局池 | ~18.9 MB | CSR coords/offsets/edges + on-network + idents + airway 名 + MORA + MSA + 导航台细节 + 等待航线 + 全局池（~1.5MB） |
| cifp | ~38 MB | 14838 机场分段程序（结构化后远小于 ~105MB 原始 CIFP 文本） |

全局池三段共用、去重后 ~1.5MB（三段各自局部池之和 ~8.7MB → −83%），是整文件比旧三文件分离
省 ~8MB 的主因。格式与取舍见 [binary-cache.zh-CN.md](binary-cache.zh-CN.md)。

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

纯搜索的分解需要一个进程内微基准（`OpenCached` 一次 + 循环 `FindRoutes` 10 城市对 × 30 轮，
`steady_clock` 只包 `FindRoutes`），并用 git 历史 commit（`2918c86` / `ee3afb4` / `f7a42c9`）
的 `core/graph/astar.*` + `yen_kshortest.*` 编译对照二进制（把这 4 个文件 checkout 到对应
commit、重编 `bf_core`、重链微基准即可，接口跨三版一致）。profile 用 gprof：`-pg -O2` 全量
编译微基准（把 `core/`+`io/` 的 .cc 与基准一起编，需 `-I build/<preset>/core` 找生成的
`version.h`），跑一轮后 `gprof <bin> gmon.out`。基准程序不入库（避免污染构建目标）。

## 8. 小结

- **启动**：缓存把冷启动 2.27s 降到 0.20s，**~11×**（换 AIRAC 才需重建，3.2s 一次性）。
- **查询**：memoize + Lawler 叠加，k=10 从 87.6ms 降到 28.5ms，**3.07×**；k=1 零退化；收益随 k 增长。
- **止步有据**：gprof 显示 A* 主循环 ~93.7% self time（遍历固有成本），据此否决了三个直觉性微优化。
- **内存/文件**：on-demand ~128MB / eager ~234MB 峰值 RSS；统一 `.bfdb` 57.3MB（CIFP 段 ~38MB）。
