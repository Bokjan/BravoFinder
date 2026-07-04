# 性能测试

> BravoFinder 的性能数据、测试方法与可复现步骤。所有数字都在同一台机器、同一份数据、
> 同一轮测量下取得，并写明口径。相关代码：`core/graph/`、`io/cache/`、`apps/cli/main.cc`。

## 1. 测试环境

| 项 | 配置 |
|---|---|
| 机器 | Apple Mac14,9（Mac mini / MacBook Pro 级） |
| 芯片 | Apple M2 Pro，10 核（物理=逻辑 10） |
| 内存 | 32 GB |
| 系统 | macOS 26.5.1（build 25F80） |
| 编译器 | Apple clang 21.0.0，`arm64-apple-darwin` |
| CMake | 4.3.4 |
| 构建配置 | `release` 预设（`-O2`），除非另注 |
| 导航数据 | X-Plane 12 native，AIRAC cycle 2601；图 V=270,821 顶点 / E=345,801 边；14838 个机场 CIFP |

> 数字与绝对时间是这台 M2 Pro 上的结果；换机器绝对值会变，但**相对关系与量级**
> （启动提速、各优化的相对贡献）应当稳定。

## 2. 启动时间：冷启动 vs 缓存加载

X-Plane 数据的解析 + 建图有固定成本（ARINC 424 解析尤重）。`bf build` 把成果落成
`.bfdb` 缓存，`bf route --db` 反序列化跳过全部解析。

**方法**：`bf route KJFK KLAX` 端到端墙钟时间（含进程启动/退出），各 5 次取稳定值。
release 构建，cycle 2601，32 核工作站。

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
| 1 | 5.48 | 5.50 | 5.39 | 1.0× |
| 3 | 17.93 | 12.16 | 10.95 | **1.64×** |
| 5 | 30.36 | 18.89 | 15.11 | **2.01×** |
| 10 | 62.78 | 37.02 | 24.61 | **2.55×** |

（单位 ms/search。）

读这张表：

- **k=1 三版几乎相同**（~5.4ms）。单次搜索里 heuristic 每顶点最多算一次、也没有 spur，
  两个优化都不改单次搜索路径——这正是它们**零退化**的证据。
- **memoize** 主要吃掉 k≥3 的重复 heuristic 计算：goals 集合在整轮 Yen 里恒定，`h(v)` 是常量
  却被数百个 spur 重复 O(goals) 扫描。k=10 从 62.8→37.0ms。
- **Lawler** 再砍掉冗余的 spur 搜索本身（只从 deviation index 起 spur）：k=10 从 37.0→24.6ms。
- **收益随 k 增长**：k 越大、spur 越多、重复越多，两个优化的空间越大。k=10 累计 **2.55×**。

> 两个优化的原理与正确性论证分别见
> [yen-lawler-optimization.zh-CN.md](yen-lawler-optimization.zh-CN.md)（Lawler + memoize）。

## 4. 已止步：profile 指向的固有成本

Lawler 之后再做了一轮带符号采样（`sample`，KJFK→KLAX k=10）。按符号归类 ~3400 样本：

| 符号 | 占比 | 说明 |
|---|---:|---|
| A* 主循环本身（边松弛 / g 更新 / 堆 push，`RunMultiSearch` 内联） | **~74%** | Yen 的固有成本 |
| malloc/free | ~3.9% | 分配，不大 |
| `DistanceTo` | ~1% | memoize 后已压下 |
| `std::set` 红黑树（Yen 候选集 + spur 禁集） | ~0.8% | 噪声级 |
| memset（O(V) 初始化） | ~0.7% | 很小 |

**结论**：两个真热点（重复 heuristic、重复 spur）已被拿掉，剩下 ~74% 是 A* 遍历的固有成本。
据此**否决了几个直觉性优化**（数据说话）：

- **栈局部 buffer / thread_local 复用搜索数组**——分配仅 ~3.9%，且省不掉躲不掉的 O(V) 初始化；
  thread_local 还违反"无全局可变状态"，线程池下每线程 ~5.4MB 永久常驻。不做。
- **Yen 禁集 `std::set` → 排序 vector**——仅 ~0.8%，噪声级收益。不做。
- **`CostOfPath` 线性找边改二分**——profile 里根本没出现，且会改 `.bfdb` 布局需 bump
  format_version。不做。

再压性能需换算法层（如 Eppstein，或 A* 遍历的预取/SIMD），属大改，不在 v3.0.0 范围。

## 5. 内存占用

**方法**：`/usr/bin/time -l` 的 maximum resident set size（进程峰值 RSS，非纯增量），
`bf route KJFK KLAX --db`。

| 模式 | 进程峰值 RSS | 缓存部分增量（DESIGN 记） |
|---|---:|---:|
| on-demand（默认） | ~94 MB | 程序缓存仅头 + 目录，+1.5 MB |
| eager（`--cifp-load eager`） | ~240 MB | 全量程序反序列化，+102 MB |

- 峰值 RSS 含图（~30MB lookup map + CSR 数组）、进程基线、以及程序缓存部分。
- on-demand 适合一次性 CLI 查询（启动省、只加载查到的机场）；eager 适合长驻服务/批量并发
  （全量常驻、之后无锁读），见 [thread-safety.zh-CN.md](thread-safety.zh-CN.md)。

## 6. 缓存文件大小

一个统一 `nav_<cycle>.bfdb` 装三段（graph + CIFP + detail），共用一个全局字符串池：

| 段 | 约占 | 内容 |
|---|---:|---|
| graph | ~16 MB | CSR coords/offsets/edges + on-network + idents + airway 名 + MORA + MSA |
| cifp | ~40 MB | 14838 机场分段程序（结构化后远小于 ~105MB 原始 CIFP 文本） |
| detail | ~2 MB | 导航台细节 + 等待航线 |
| 全局池 | ~1.5 MB | 三段共用，去重后（三段各自局部池之和 ~8.7MB → −83%） |

整文件实测 ~57 MB（cycle 2601），比旧三文件分离省 ~8MB（全局池去重）。格式与取舍见
[binary-cache.zh-CN.md](binary-cache.zh-CN.md)。

## 7. 如何复现

启动与端到端（用发布的 CLI）：

```bash
cmake --preset release && cmake --build --preset release
bf build navdata -o /tmp/nav.bfdb              # 一次性建缓存，计时见 bf build

# 冷启动 vs 缓存，各跑几次
for i in 1 2 3 4 5; do /usr/bin/time -p bf route KJFK KLAX --data navdata >/dev/null; done
for i in 1 2 3 4 5; do /usr/bin/time -p bf route KJFK KLAX --db /tmp/nav.bfdb >/dev/null; done

# 内存
/usr/bin/time -l bf route KJFK KLAX --db /tmp/nav.bfdb >/dev/null
/usr/bin/time -l bf route KJFK KLAX --db /tmp/nav.bfdb --cifp-load eager >/dev/null
```

纯搜索的分解需要一个进程内微基准（`OpenCached` 一次 + 循环 `FindRoutes` 10 城市对 × 30 轮，
`steady_clock` 只包 `FindRoutes`），并用 git 历史 commit（`2918c86` / `ee3afb4` / `f7a42c9`）
的 `core/graph/astar.*` + `yen_kshortest.*` 编译对照二进制。基准程序不入库（避免污染
构建目标）；上面的方法描述足以重建。

## 8. 小结

- **启动**：缓存把冷启动 2.27s 降到 0.20s，**~11×**（换 AIRAC 才需重建，3.2s 一次性）。
- **查询**：memoize + Lawler 叠加，k=10 从 62.8ms 降到 24.6ms，**2.55×**；k=1 零退化；收益随 k 增长。
- **止步有据**：profile 显示剩余 ~74% 是 A* 遍历固有成本，据此否决了三个直觉性微优化。
- **内存/文件**：on-demand ~94MB / eager ~240MB 峰值 RSS；图缓存 17.9MB + 程序缓存 46.6MB。
