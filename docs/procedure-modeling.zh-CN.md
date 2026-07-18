# 程序建模与航路网衔接：ARINC 424 / CIFP

> 这是 BravoFinder 工作量最大、也最能体现「真实」的部分：解析真实的 SID/STAR/进近程序， 并把机场正确接入航路网。面向想理解「机场是怎么连上航路的」的读者。相关代码： `lib/io/loaders/xplane12/cifp/`（解析器 + 衔接器）、`lib/core/domain/procedure.h`。

## 1. 为什么机场不能「直连最近航路点」

一个自然的想法：从机场拉一条直线到最近的在网航路点，接入航路网。**这在真实数据上行不通。**

M1 阶段的实测发现（DESIGN §9）：机场坐标附近最近的航路点，**几乎全是终端区进近航点** ——它们只通过 SID/STAR 程序连接，本身不在航路网（enroute network）里。以 KJFK 为例， 周边 96 个最近航点里**只有 1 个**真正在航路网中。若直连最近航点，机场会被困在这些终端 「死胡同」上，根本算不出航路。

结论：**机场必须靠真实程序接入航路网**。这就是为什么程序建模不是锦上添花，而是能否算出 航路的前提。

## 2. ARINC 424 / CIFP：生的 path terminator

X-Plane 的 `CIFP/<ICAO>.dat` 是 ARINC 424 派生的终端程序格式，每个机场一个文件。每行是 `记录类型:序号,逗号分隔字段...;`，记录类型有 SID / STAR / APPCH / RWY / PRDAT。一条命名 程序（如 `DEEZZ5`）由多条 **leg** 组成，每条 leg 有一个 **path terminator**（航段类型）， 决定这段怎么飞、以什么结束。

全量 14838 机场的语料里用到 **23 种** path terminator（`lib/core/domain/procedure.h` 全部识别， 未知码归为 `kUnknown`，绝不静默丢 leg）。它们分两类：

- **「飞到定点」型**（TF/IF/DF/CF）：终点是确定的航路点，能解析成图顶点。占绝大多数。
- **「飞航向/弧/高度/等待」型**（VA/VM/CA/VI/VR/FM/RF/HM/…）：终点不是固定航点（飞到某高度、 航向截获、等待），无法直接对应一个顶点。

BravoFinder 解析并**保留每条 leg 的完整结构化信息**（path terminator、航向、距离、高度限制， 以及 RNP / 转向方向 / 速度限制），即使暂时不参与建图（用于输出展示：`bf query procedure ICAO/名称` 按程序名列出每条 leg 的这些字段）。解析器见 `cifp_parser.cc`；三个 loader（xplane12 / dfd1 / dfd2） 都填这些字段，缺席即留空值（`optional` 语义的 0 / `'\0'`），不按 `source_loader` 分支。

## 3. 「飞航向/弧/等待」型 leg：全量复核后的重新定性

DESIGN §4.4 最初设想：非定点 leg 会挡住程序接入，需要用航向+距离「折叠」成等效边。但对 **全量 14838 机场（192283 条程序）**统计后，这个担心几乎不成立（DESIGN §9 ②）：

- **STAR：0%** 全非定点；**SID：仅 2.43%** 全非定点。
- 那 2.43% 是**雷达引导离场**（如 KPHL PHL4：VA→VM），本就没有固定衔接 fix——**不该伪造** 一个出来。
- 可达性已由定点 fix 覆盖：STAR 100%、SID 97.6% 都能靠定点 fix 接入航路网。

剩下的只是 SID 首段的 seed 精度问题，而其中 87.7% 的非定点首段是 **course-only 无距离** （CA/VA/VM），物理上无法几何推算——要算等效位移就得引入运动学假设（爬升率/转弯率/风）， 那属于后期「几何航迹推算」里程碑。所以第一阶段**不折叠等效边**；对雷达引导离场，如实标注 「RADAR VECTORS」而非伪造衔接点（见 §6）。

## 4. 衔接 fix 选点：暴露「每一个」在网 fix

一条 SID 可以在它经过的**任意一个在网 fix** 处把飞机交给航路网——「从 XX 航路点加入航路」是 常规操作，不是只能用它的最后一个 fix.STAR 对称：可以在它经过的任意在网 fix 处接手。

所以 `ProcedureConnector`（`procedure_connector.cc`）不再只暴露一个衔接点，而是把程序经过的 **全部在网 fix** 都作为候选 `Connection` 暴露出来，交给多源搜索择优。关键设计：

- **按 vertex 去重、聚合所有 ProcedureRef**：多条程序共享同一个 fix 时，列出全部可换的 SID/STAR，无需重复搜索。
- **seed = 沿程序公布折线累计**（`FixHit.cumulative_nm`）+ 仅对「机场↔记录端点」未测段补直线。 沿航迹累计意味着：一个绕远才到达的 fix 会得到**更大（更诚实）**的 seed，而不是它的直线 距离——搜索因此自然偏向更近的在网 fix。
- **在网判定分方向**：SID 是「飞到 fix 再沿航路飞出」，衔接 fix 需要有**出边** （`GraphBuilder::HasOutbound`）；STAR 是「沿航路飞进 fix 再由程序接手」，衔接 fix 需要有 **入边**（`HasInbound`）。二者不能共用「有出边」这一个判定——否则只作为 forward-only 航段 终点的 STAR 入口门户（只有入边、无出边）会被误判为脱网。典型：VHHH 的 `ABEY` 系列 STAR 入口 `ABBEY` 仅由 forward-only 的 `FISHA→ABBEY` 到达；用出边判定会跳过它、迫使 RJTT→VHHH 绕到西南 的 SIKOU 接 `SIER7C`；改用入边判定后走 `…FISHA→ABBEY` + `ABEY` STAR，省约 330 NM。 （`OnNetwork` 现为入边∪出边的并集，仅用于 `bf query` 的 `[on-network]` 展示。）

实测效果：KDEN→KLAX 现在从 BASET5 的公共段 **DOWNE 进场**（末段 STAR leg 仅 14.2 NM）， 而不再被迫接 ~260 NM 外的 PGS。

## 5. 多源 K-shortest：候选可用不同程序

把每个衔接 fix 当作一个带 seed 的搜索端点后，K 条候选就不再局限于「固定一对 fix 之间变航路」， 而是**每条候选都能走不同的 SID/STAR 衔接 fix**。`FindKShortestPathsMulti` （`lib/core/graph/yen_kshortest.cc`）用概念超源/超汇在多源 A* 之上做 Yen：

- source 级 spur 禁掉已用的起始 fix、重跑多源搜索 → 换一个衔接 fix/程序；
- 每条候选用 `CostOfPathMulti` 端到端重算（含两端 seed），使不同 fix 对之间仍能正确排序。

实测：KSEA→KLAX 的 K 候选里 STAR 从 KIMMO3 切换到 WAYVE1，证明跨程序备选端到端可用。

> 这里的 Yen 在大图上的性能优化（Lawler + heuristic memoization）单独成文： [yen-lawler-optimization.zh-CN.md](yen-lawler-optimization.zh-CN.md)。

## 6. 语义诚实：区分三种「回退直飞」

当程序无法把机场接入航路网时会回退到 DCT（直飞），但**回退的原因不同**，混为一谈会误导。 引擎用一个对称的 `ConnectionKind` 区分（Route 的 `dep_connection`/`arr_connection`）：

- `kProcedure`——真的用了程序接入；
- `kRadarVectors`——机场**发布了** SID/STAR，但没有一个能到达在网 fix（雷达引导离场）；
- `kDirect`——机场**根本没有**程序数据，纯 DCT 回退。

CLI/JSON 会显式标注 「RADAR VECTORS」，把「雷达引导」和「缺数据」两种情况区分开——这是「语义 诚实」而非假装有一条程序。判断依据是 `has_procedures`（该侧是否发布了 SID/STAR）与 `used_procedures`（是否真的接上了）两个标志。

## 7. 单文件部署：CIFP 分段缓存

14838 个 CIFP 散文件不便部署。引擎把它们打包成统一 `.bfdb` 里的一个**分段 CIFP 段**（与 graph、 detail 同处一个文件、共用全局字符串池），按需加载单机场程序段，启动仍是毫秒级。这属于缓存 工程，单独成文见 [binary-cache.zh-CN.md](binary-cache.zh-CN.md)。

## 8. 小结

机场接入航路网这条链，环环相扣：

1. 机场附近全是终端死点 → **必须靠程序接入**（不能直连最近航点）；
2. 解析 ARINC 424 全 23 种 path terminator，保留完整结构；
3. 全量复核证明非定点 leg 几乎不挡接入，**不伪造等效边**；
4. 暴露程序经过的**每个**在网 fix 作候选，沿航迹 seed 引导择近；
5. 多源 K-shortest 让候选走不同程序；
6. 无法接入时**如实区分**雷达引导 vs 缺数据。
