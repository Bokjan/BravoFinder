# Yen K-shortest 的 Lawler 优化

> 面向想读懂 `lib/core/graph/yen_kshortest.cc` 的开发者。这段代码用了一个不算常规的 算法优化（Lawler），且作用在一个非标准的**多源多汇**变体上，正确性论证不显然， 故单独成文。代码注释是英文，本文用中文解释来龙去脉。

## 1. 背景：K-shortest 为什么慢

BravoFinder 用 Yen 算法求前 K 条候选航路（`FindKShortestPathsMulti`，生产路径唯一入口 `lib/io/nav_database.cc`）。Yen 的骨架是：

- **A 集**：已接受的路径（最终结果）。
- **B 集**：候选路径，按成本排序。
- 每接受一条路径后，对它的**每个节点**做一次 「spur 搜索」（从该节点重新搜到终点， 并禁掉会重复已有路径的边），把结果塞进 B 集，再从 B 集取最便宜的作为下一条。

在真实数据（V=270,821 顶点）上实测：

| k | ms/search（优化前） |
|---|---|
| 1 | 0.93 |
| 3 | 23.5 |
| 10 | 103 |

k=1→3 有 **25 倍断崖**。瓶颈在 Yen 循环：k=10、路径 ~30 节点时约 **270 次 spur**， 每次 spur 是一次完整的图搜索（~1–5ms）。而这些 spur 里**大量是重复计算**。

> 注：另有一个正交优化——多源 heuristic 的 memoization（见 commit `ee3afb4`）， 把每次 spur 内部的 O(goals) haversine 扫描缓存掉。本文只讲 Lawler。

## 2. Lawler 优化：只从 deviation node 起 spur

**核心观察**：第 k 轮对上一条路径 `A^(k-1)` 的**全部**节点做 spur，但 `A^(k-1)` 与更早 的路径共享一段前缀，那段前缀上的 spur 在更早的轮次里**已经算过、结果已进 B 集**。重复。

**Lawler 的改法**：记住每条候选是在父路径的**第几位** spur 生成的——这个索引叫 **deviation node（偏离节点）**。一条候选被接受后，下一轮**只从它的 deviation index 起** spur， 更早的位置跳过。

**为什么正确**：deviation index 之前的任何 spur 位置，其 root 前缀与上一轮处理过的某个 root 完全相同；而每一轮都会穷尽该 root 下的所有偏离 → 这些更早位置只会重新生成上一轮 已考虑过的路径。所以跳过它们**不丢任何新候选**。Lawler 是**纯去重**：接受的路径集合与 顺序**逐条不变**，只是省掉冗余计算。

**三个正确性前提**（对照本仓库实现）：

1. **B 集跨轮持久化，不清空。** ✅ `candidates` 定义在 `kth` 循环外。
2. **edge-ban 用「同前缀才禁」语义、跨轮一致。** ✅ 见 `std::equal(root..., p.vertices...)` —— 只禁与当前 root 同前缀的已知路径的分叉边，**不是**禁全部已接受路径。这一点在引入 Lawler 之前就写对了，是 Lawler 能成立的基础。
3. **每条候选记住自己的 deviation index，接受后据此决定下轮 spur 起点。** 这是本次新增： `Candidate` 加了 `int deviation` 字段（**不参与排序键**，只是元数据），接受时用 `last_deviation = best->deviation` 传给下一轮。

## 3. 我们的非标准变体：多源多汇 + 超源

`FindKShortestPathsMulti` 不是教科书的单源单汇 Yen。它要在**多个起始衔接 fix**（SID 交给 航路网的候选点）和**多个终止 fix**（STAR 接手点）之间求 K 条，每个端点带一个 seed 成本 （程序段估算距离）。

实现上用一个**概念超源**：循环索引 `i = -1` 表示「在超源处偏离」，即重跑多源搜索、禁掉 已用过的起始 fix，从而换一个不同的 SID/STAR 入口。`i ≥ 0` 是普通的单源→多汇 spur。

**Lawler 对这个变体仍成立的论证**：把「超源 → 首个 fix」看作路径的第 `-1 → 0` 段，则多源 Yen 与标准 Yen **同构**，deviation node 概念平移即可，唯一区别是 deviation index 的取值域 要**含 -1**。所以：

- 首条路径来自初始多源搜索，概念上在超源偏离 → `last_deviation` 初值 **-1**（整条都要 spur）。
- 一条候选若在 `i=-1` 生成，deviation = -1；在 `i≥0` 生成则 deviation = i。
- 下一轮 `for (int i = last_deviation; ...)` 从该值起，自然涵盖 -1 的情形。

超源无实体、不进 `banned_nodes`，唯一特殊处理就是 `i<0` 时走「重跑多源 + 禁起始 fix」分支 ——这与标准 Lawler 的「从最小 deviation index 起」完全一致。

## 4. 实现要点（`lib/core/graph/yen_kshortest.cc`）

- `Candidate` 加 `int deviation`；**成本可忽略**：多一个 int（4B）相对已有的 `vertices` 向量微不足道，且不参与 `operator<`（排序键仍是 cost → vertices），红黑树开销不变。
- 单源 `FindKShortestPaths`：`last_deviation` 初值 0；内层 `for (i = last_deviation; ...)`。
- 多源 `FindKShortestPathsMulti`：`last_deviation` 初值 -1；`add_candidate` 多带一个 `deviation` 参数。
- **线程安全契约 B 不破**：`last_deviation` 是搜索/Yen 调用的函数局部 int，ban 集仍按值 捕获进 `std::function`（自包含、可跨线程持有）。tsan 已验。

## 5. 验证：差分对拍（golden-reference testing）

Lawler 是纯优化 → 前后结果必须逐条一致。这一点**必须**被测试锁死，否则「少算了一条候选」 这种 bug 极难察觉。`tests/unit/yen_test.cc` 用两层验证：

1. **固定黄金签名**：两个 4×3 lattice 图，硬编码优化前捕获的完整候选序列（cost + 顶点 列表），改动后必须逐字节复现。
2. **随机差分对拍**：测试文件内置一个**独立的朴素 Yen 参考实现**（从 i=0 / i=-1 全 spur、 无 deviation 跟踪，即「没有 Lawler 的教科书 Yen」），在 **400 组随机小图**（单源 200 + 多源 200，k=1..10，固定种子可复现）上与 Lawler 版对拍，要求 tie-break **完全一致** （cost 后按 vertices）、结果逐条相同。两个独立实现互验，覆盖面远超固定样例。

两层的**甄别力**都验证过：故意把 `last_deviation` 改错（`+1` 跳过一个 spur 位）会让对拍 立即变红。

## 6. 收益

真实数据 10 城市对 × 20 次基准，Lawler 单独贡献（同次运行对照）：

| k | 优化前 | +Lawler | 加速 |
|---|---|---|---|
| 3 | 12.3 | 11.2 | 1.09× |
| 5 | 19.1 | 15.3 | 1.25× |
| 10 | 38.0 | 24.8 | **1.53×** |

收益随 k 增长（k 越大、重复 spur 越多）。与 heuristic memoization 叠加，k=10 从最初的 ~63ms 降到 ~25ms，累计约 **2.5×**。

## 7. 参考

- Yen, J. Y. (1971). 「Finding the K Shortest Loopless Paths in a Network.」
- Lawler, E. L. (1972)。 对 Yen 的改进（避免重复 spur 计算）。
- 维基：「Yen's algorithm」（含 Lawler 修改的描述）。
