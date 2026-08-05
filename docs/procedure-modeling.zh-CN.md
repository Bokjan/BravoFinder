# 程序建模与航路网衔接：ARINC 424 / CIFP

> 这是 BravoFinder 工作量最大、也最能体现「真实」的部分：解析真实的 SID/STAR/进近程序，并把机场正确接入航路网。面向想理解「机场是怎么连上航路的」的读者。相关代码： `libs/engine/io/loaders/xplane12/cifp/`（解析器 + 衔接器）、`libs/engine/core/domain/procedure.h`。

## 1. 为什么机场不能「直连最近航路点」

一个自然的想法：从机场拉一条直线到最近的在网航路点，接入航路网。**这在真实数据上行不通。**

M1 阶段的实测发现（DESIGN §9）：机场坐标附近最近的航路点，**几乎全是终端区进近航点** ——它们只通过 SID/STAR 程序连接，本身不在航路网（enroute network）里。以 KJFK 为例，周边 96 个最近航点里**只有 1 个**真正在航路网中。若直连最近航点，机场会被困在这些终端「死胡同」上，根本算不出航路。

结论：**机场必须靠真实程序接入航路网**。这就是为什么程序建模不是锦上添花，而是能否算出航路的前提。

## 2. ARINC 424 / CIFP：生的 path terminator

X-Plane 的 `CIFP/<ICAO>.dat` 是 ARINC 424 派生的终端程序格式，每个机场一个文件。每行是 `记录类型:序号,逗号分隔字段...;`，记录类型有 SID / STAR / APPCH / RWY / PRDAT。一条命名程序（如 `DEEZZ5`）由多条 **leg** 组成，每条 leg 有一个 **path terminator**（航段类型），决定这段怎么飞、以什么结束。

全量 14838 机场的语料里用到 **23 种** path terminator（`libs/engine/core/domain/procedure.h` 全部识别，未知码归为 `kUnknown`，绝不静默丢 leg）。它们分两类：

- **「飞到定点」型**（TF/IF/DF/CF）：终点是确定的航路点，能解析成图顶点。占绝大多数。
- **「飞航向/弧/高度/等待」型**（VA/VM/CA/VI/VR/FM/RF/HM/…）：终点不是固定航点（飞到某高度、航向截获、等待），无法直接对应一个顶点。

BravoFinder 解析并**保留每条 leg 的完整结构化信息**（path terminator、航向、距离、高度限制，以及 RNP / 转向方向 / 速度限制），即使暂时不参与建图（用于输出展示：`bf query procedure ICAO/名称` 按程序名列出每条 leg 的这些字段）。解析器见 `cifp_parser.cc`；所有 loader（xplane12 / dfd1 / dfd2 / fenix）都填这些字段，缺席即留空值（`optional` 语义的 0 / `'\0'`），不按 `source_loader` 分支。

## 3. 「飞航向/弧/等待」型 leg：全量复核后的重新定性

DESIGN §4.4 最初设想：非定点 leg 会挡住程序接入，需要用航向+距离「折叠」成等效边。但对 **全量 14838 机场（192283 条程序）**统计后，这个担心几乎不成立（DESIGN §9 ②）：

- **STAR：0%** 全非定点；**SID：仅 2.43%** 全非定点。
- 那 2.43% 是**雷达引导离场**（如 KPHL PHL4：VA→VM），本就没有固定衔接 fix——**不该伪造** 一个出来。
- 可达性已由定点 fix 覆盖：STAR 100%、SID 97.6% 都能靠定点 fix 接入航路网。

剩下的只是 SID 首段的 seed 精度问题，而其中 87.7% 的非定点首段是 **course-only 无距离** （CA/VA/VM），物理上无法几何推算——要算等效位移就得引入运动学假设（爬升率/转弯率/风），那属于后期「几何航迹推算」里程碑。所以第一阶段**不折叠等效边**；对雷达引导离场，如实标注「RADAR VECTORS」而非伪造衔接点（见 §6）。

## 4. 衔接 fix 选点：只接程序**发布的**入口/出口

程序和航路网的交接点不是「随便哪个 fix 都行」，而是航图上公布的那一个：

- **STAR 从它的 Initial Fix 进**（`path_term == IF`，`PathTerminator::kIF`）。真实数据里这一列填得很干净——X-Plane CIFP 的 KLAX 116/116 条 STAR 每个过渡首腿都是 `IF`；三个 SQLite loader（dfd1 / dfd2 / Fenix）的 STAR 过渡首腿有 **99.8%** 是 `IF`（40798/40870、44102/44182）。
- **SID 从它的末端 fix 出**——即「最后一条终止于定点 fix 的航段」（`LastFixBearingLeg`）。SID 没有专门的出口列，必须派生；**不能用 IF**：SID 里的 `IF` 标的是「某个过渡从哪个 fix 分叉开始」，是记录的另一端。派生结果与航图上**加粗的过渡末端 fix** 一致（KLAX `SKWRL2` 的 5 个过渡逐一核对成立）。

`ProcedureConnector`（`procedure_connector.cc`）因此只把这些**发布衔接点**暴露成候选 `Connection`，交给多源搜索择优。关键设计：

- **按 vertex 去重、聚合所有 ProcedureRef**：多条程序共享同一个 fix 时，列出全部可换的 SID/STAR，无需重复搜索。
- **seed = 沿程序公布折线累计**（`FixHit.cumulative_nm`）+ 仅对「机场↔记录端点」未测段补直线。沿航迹累计意味着：一个绕远才到达的 fix 会得到**更大（更诚实）**的 seed，而不是它的直线距离。
- **在网判定分方向，且与「是否发布」正交**：SID 是「飞到 fix 再沿航路飞出」，衔接 fix 需要有**出边**（`GraphBuilder::HasOutbound`）；STAR 是「沿航路飞进 fix 再由程序接手」，衔接 fix 需要有**入边**（`HasInbound`）。二者不能共用「有出边」这一个判定——否则只作为 forward-only 航段终点的 STAR 入口门户（只有入边、无出边）会被误判为脱网。典型：VHHH 的 `ABEY` 系列 STAR 入口 `ABBEY` 仅由 forward-only 的 `FISHA→ABBEY` 到达；用出边判定会跳过它、迫使 RJTT→VHHH 绕到西南的 SIKOU 接 `SIER7C`；改用入边判定后走 `…FISHA→ABBEY` + `ABEY` STAR，省约 330 NM。（`OnNetwork` 现为入边∪出边的并集，仅用于 `bf query` 的 `[on-network]` 展示。）
- **机场级回退**：若整个机场该侧**一个在网发布衔接点都没有**（cycle 2601：到达侧 30 个机场、离场侧 6 个），退回「暴露该程序经过的全部在网 fix」，而不是直接掉到 DCT——保住程序语义。回退刻意做在**机场级**而非**逐程序级**：按程序回退会在另外 19 个到达 / 17 个离场机场重新放进近场末端 fix（doorstep 机场 3 → 22），正是这条规则要防的退化。代价是「某条具名程序没有在网发布衔接点、而同机场其他程序有」时，它不能被 `--star` / `--sid` 按名选中。

### 为什么必须限制到发布衔接点

早期模型是「暴露程序经过的**每一个**在网 fix」，理由是「从 XX 航路点加入航路」属常规操作。但它有一个结构性缺陷：程序里若存在一个**贴着跑道**（沿程序到跑道 <1 NM）又恰好在航路网上的 fix，A\* 会把它当成≈免费的落地点，用航路一路飞到机场门口，把整条 SID/STAR 压成零长度 stub。典型是 KJFK→YSSY 的 `…B450 TESAT STAR YSSY`（`arr 0.3 NM`；TESAT 是 MARLN5 的末端 `TF` fix，落在 B450 上）。这类路线**总长反而更短**（8706.9 vs 8744.9 NM），所以优化器没算错——是 seed 定价把「航路飞到贴场 fix」标成了近乎免费。

全量数据说明这个病灶几乎完全落在「末端 fix」这一类上：cycle 2601 里 STAR 侧 `seed<1 NM` 的 1523 个 per-fix 命中中，**1521 个是 `TF`/`CF`**（末端 fix 类），`IF` 只有 1 个；发布 `IF` 的 seed 中位数是 **64 NM**。所以「只接发布入口/出口」在结构上就绕开了它：到达侧 doorstep 机场从 255 降到 **3**，离场侧 344 → 124，且残留全是 `total=0` 的退化单 fix 程序（没有程序主体可绕，不是同一个病）。这取代了早先按 seed 阈值事后剔除的读侧 filter（经验阈值 T=1.0 / fb=20.0），后者依赖「次落点恰好落在 [1,20] NM」才敢下手，遇到 WAAA 那种 2.2 NM 入口 + 147.9 NM 跳空就无从判断。

代价是**合法的中间 fix 交接也一并取消**了——KLAX 的到达候选从 57 个降到 33 个，`DOWNE`（14.2 NM）、`SMO`（4.7 NM）这类中途 `TF` fix 不再是候选，最近的发布入口变成 `SADDE`（20.2 NM）。实测这不是回归：KDEN→KLAX 现在走 `…Q88 HAKMN STAR KLAX`（total 765.3 NM），比强制经 DOWNE 的 800.8 NM **更短**——沿程序折线累计的 seed 已经把「入口远」的成本诚实计入，搜索会自己权衡。全量抽样（1984 对）路线成功率逐字不变（1963/1984），中位总距离 +0.0 NM。

## 5. 多源 K-shortest：候选可用不同程序

把每个衔接 fix 当作一个带 seed 的搜索端点后，K 条候选就不再局限于「固定一对 fix 之间变航路」，而是**每条候选都能走不同的 SID/STAR 衔接 fix**。`FindKShortestPathsMulti` （`libs/engine/core/graph/yen_kshortest.cc`）用概念超源/超汇在多源 A* 之上做 Yen：

- source 级 spur 禁掉已用的起始 fix、重跑多源搜索 → 换一个衔接 fix/程序；
- 每条候选用 `CostOfPathMulti` 端到端重算（含两端 seed），使不同 fix 对之间仍能正确排序。

实测：KSEA→KLAX 的 K 候选里 STAR 从 KIMMO3 切换到 WAYVE1（分别经发布入口 LHS / LOPES），证明跨程序备选端到端可用。

> 这里的 Yen 在大图上的性能优化（Lawler + heuristic memoization）单独成文： [yen-lawler-optimization.zh-CN.md](yen-lawler-optimization.zh-CN.md)。

## 6. 无 STAR 机场：DCT-to-IAF（终端过渡）

不少小机场**没有公布 STAR**，但有进近（approach）。原先到达侧在 STAR 候选为空后直接走 DCT fallback——按大圆取最近 on-network fix，**对 IAF 无感知**（例如 KTVL 曾出现 `… MARRI DCT KTVL`，绕过在网的 HETRY）。

从 v3.24.0 起，到达侧在「无 STAR（或 STAR 候选全空）且未指定命名 `--star`」时，会尝试把 **approach 的 IAF** 接进到达候选（`BuildApproachArrival`）：

- **只接 gate**：approach 的 gate 是 `path_term == IF`（=IAF）；不要把 FAF / MAPT / 复飞点当成连接点。
- **filed string 诚实**：`star` 字段留空；航路串止于最后一个**在网** fix，形如 `… <fix> DCT ARR`。进近名、IAF、磁航向等只进 detail 元数据（`approach` / `approach_iaf` / `approach_bearing` / `approach_options`），**不**把进近标识塞进 ATS Field 15 风格的 string（那不是「合成 STAR」）。
- **off-net IAF 用代理 goal**：IAF 若无入边，不能当 search goal。引擎不为它们加虚拟边；改为在 IAF 附近取若干在网代理点，seed 含 `|代理→IAF| + IAF→MAPT 程序体`，与在网 IAF 同池比价。
- **seed 止于 MAPT**：长度按 IAF→MAPT（Waypoint Description Code 的 MAPT 位），不含复飞段。
- **命名 `--star` 不匹配仍报错**，不静默落到 IAF。

有 STAR 的机场行为不变。若某机场**有** STAR 但其入口 fix **全部**脱网，那是另一条「STAR splice」路径（另开跟踪），不在本节。

## 7. 语义诚实：区分四种到达/离场连接

当程序无法（或不该）把机场写成 SID/STAR 接入时，回退原因不同，混为一谈会误导。引擎用 `ConnectionKind` 区分（Route 的 `dep_connection`/`arr_connection`）：

- `kProcedure`——真的用了 SID/STAR 接入（`sid`/`star` 有名）；
- `kTerminalTransition`——**无 STAR**、用了进近 IAF 的 DCT-to-IAF（见 §6）；`star` 空，元数据带 approach；
- `kRadarVectors`——机场**发布了** SID/STAR，但没有一个能到达在网 fix（雷达引导）；
- `kDirect`——该侧**没有**可用的 SID/STAR/approach 衔接，纯 DCT 回退。

CLI/JSON 会显式标注「RADAR VECTORS」或「APCH PROC」等，把「雷达引导 / 终端过渡 / 缺数据」分开——这是语义诚实，而非假装有一条 STAR。判断依据是该侧是否发布了程序、以及本次是否真的用了 STAR 或 approach。

## 8. 单文件部署：CIFP 分段缓存

14838 个 CIFP 散文件不便部署。引擎把它们打包成统一 `.bfdb` 里的一个**分段 CIFP 段**（与 graph、 detail 同处一个文件、共用全局字符串池），按需加载单机场程序段，启动仍是毫秒级。这属于缓存工程，单独成文见 [binary-cache.zh-CN.md](binary-cache.zh-CN.md)。

## 9. 小结

机场接入航路网这条链，环环相扣：

1. 机场附近全是终端死点 → **必须靠程序接入**（不能直连最近航点）；
2. 解析 ARINC 424 全 23 种 path terminator，保留完整结构；
3. 全量复核证明非定点 leg 几乎不挡接入，**不伪造等效边**；
4. 到达/离场只在**发布衔接点**交接（STAR 的 IF、SID 的末 fix），机场级回退兜底；
5. 多源 K-shortest 让候选走不同程序；
6. 无 STAR 时用 **DCT-to-IAF** 接进近，filed 仍写 DCT、进近只进元数据；
7. 无法接入时**如实区分**程序 / 终端过渡 / 雷达引导 / 缺数据。
