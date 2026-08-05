# 可插拔约束层：把「合规」做成一等公民

> 航路的「能不能飞」从来不是单一判据：高度带、地形、方向、国别惯例、ATC 偏好，各自独立又要一起生效。这篇讲 BravoFinder 怎么把它们收进一个统一的约束框架，以及一条真实需求（issue #22 的国别航路过滤）如何暴露出「匹配粒度」这个比框架本身更棘手的问题。相关代码：`libs/engine/core/constraints/`、`libs/engine/core/graph/astar.cc`、`libs/engine/io/nav_database_routing.cc`。
>
> 前置：[合规航路引擎](compliant-routing.zh-CN.md) 讲了为什么需要约束（地理最短 ≠ 可飞），本文讲这个框架长什么样、怎么扩展、扩展时会踩什么。

## 1. 三态裁决：一个接口装下硬禁止与软偏好

约束层的全部接口就是一个纯虚函数：

```cpp
class Constraint {
 public:
  virtual EdgeVerdict Evaluate(const EdgeContext& ctx, const RouteRequest& request) const = 0;
};
```

裁决结果是三态的：

```cpp
struct EdgeVerdict {
  bool allowed = true;      // false => 硬过滤：这条边不可用
  double extra_cost = 0.0;  // 软惩罚：加到这条边的通行成本上

  static EdgeVerdict Allow();
  static EdgeVerdict Block();
  static EdgeVerdict Penalize(double cost);
};
```

**为什么必须有「软」这一态**，而不是全做成硬过滤？因为现实中的规则大多不是二值的。「优先走高空航路」不等于「禁止低空航路」——跨层衔接在真实飞行里很常见；「X/V 航路 ATC 严查、尽量避开」也不等于「绝不可用」——当它是唯一通路时仍得飞。硬过滤会把这类偏好变成不可逆的连通性断裂，让本可给出次优解的查询直接返回「无航路」。

软惩罚的另一重价值是**它保留了成本比较的语义**。搜索不是在「合规集合」里挑最短，而是在一个被重新定价的图上找最优：一条被罚了 50% 的航路，如果它比替代方案短 60%，仍然会胜出——这正是我们想要的判断，而硬过滤剥夺了做这个判断的机会。

组合规则由搜索统一处理，约束之间互不知情（`astar.cc` 的 `EdgeAllowed`）：

```cpp
for (const Constraint* c : options.constraints) {
  const EdgeVerdict v = c->Evaluate(ctx, *options.request);
  if (!v.allowed) {
    return false;         // 任一约束 block => 边不可用，短路
  }
  extra_cost += v.extra_cost;   // 软惩罚累加
}
```

**任一 block 则禁用、软惩罚相加**。累加而非取最大值，是因为每个约束陈述的是一个**独立的规避理由**：「这是低空航路而你想飞高空」和「这是中国的 J 航路」是两个应当叠加的顾虑。累加也让惩罚总量可以超过边长本身（罚到 1.5 倍、2 倍），这不是 bug 而是刻意保留的表达力——用户可以把强度调到接近禁止，同时保住图的连通性。

## 2. 可采纳性：软惩罚为什么不破坏 A\* 的最优性

这是软惩罚方案最容易被质疑的地方，值得说清。A\* 要求启发函数 h(v) 是**真实剩余代价的下界**才能保证最优。我们的 h 是大圆距离（实际是弦长，见 [routing-basics](routing-basics.zh-CN.md)），而软惩罚**增加**了边的有效代价——所以真实剩余代价只会变大，h 只会变得更保守，下界关系不受影响。

这条推理依赖一个硬性前提：**惩罚必须非负**。一个负惩罚（「这条航路特别好，给它减价」）会让真实代价降到 h 之下，A\* 的最优性立刻失效。所以每个软约束都在接口层守住这一点：`penalty_fraction` 校验 `>= 0`（CLI 与 JSON 两层都拒绝负值），`RandomizeConstraint` 的抖动取 `[0,1)` 区间的哈希值乘非负系数。想表达「偏好 A」时，正确做法是**惩罚非 A**，而不是奖励 A——两者对搜索的相对效果相同，但只有前者保持可采纳。

## 3. `EdgeContext`：约束能看到什么

```cpp
struct EdgeContext {
  GraphEdge edge;
  Coordinate from_coord;
  Coordinate to_coord;
  int from = -1;  // 源顶点索引（edge.to 是目标）
};
```

四个成员，全部按值持有。**按值而非引用**是有意的：`EdgeContext` 在 A\* 松弛的热路径上每条边构造一次，三个 16 字节 POD 加一个 int 的拷贝是寄存器级的、亚纳秒的；换成引用成员则要为每次字段读付一次间接寻址（MORA 约束会多次读两个端点），实测无收益。按值还让它免于生命周期陷阱——可以绑定到临时的 edge/Coordinate 值（约束单测正是这么做的）而不悬垂。

`from` 这个成员来得较晚，加它的原因值得记：**`GraphEdge` 只存 `to`**。图是 CSR 存储的，源顶点是行索引、隐含在遍历位置里，边结构体本身不存（16 字节已被 `to`/`distance_nm`/`airway_id`/`base_fl`/`top_fl`/`level` 占满，`static_assert` 钉死）。在需要「航段的**两个**端点各在哪个区域」之前，没有约束关心源顶点，所以这个字段一直不存在。issue #22 的国别过滤要求判断「leg 任一端点在区内」，才第一次需要它——`astar.cc` 的两处构造点（`SelectEdge` 与松弛循环）都已持有源顶点，顺着既有的 `EdgeAllowed` 接缝传进来即可。

这是框架扩展的一个典型形态：**接口不动，上下文加字段**。加成员比让每个约束自己想办法反查源顶点要诚实，也比把 `from` 塞进 `GraphEdge`（会破坏 16 字节不变量、膨胀整个边数组）划算。

## 4. 六个内置约束

`FindRoutes`（`nav_database_routing.cc`）按请求内容组装激活哪些约束——**没设的字段不付任何代价**，约束不进链：

| 约束 | 类型 | 判据 | 激活条件 |
|---|---|---|---|
| `AltitudeBandConstraint` | 硬 | 巡航高度区间与航段 `[base_fl, top_fl]` 是否重叠 | `altitude` 已设 |
| `MoraConstraint` | 硬 | 巡航高度是否低于沿线 MORA | `altitude` 已设 |
| `LevelPreferenceConstraint` | 软 | 航段 level 是否匹配高/低空偏好 | `level != kNone` |
| `AvoidWaypointConstraint` | 硬 | 目标顶点是否在避让集合内 | `avoid_waypoints` 非空 |
| `AirwayRuleConstraint` | 硬+软 | region × designator 规则匹配 | `airway_rules` 非空 |
| `RandomizeConstraint` | 软 | `hash(seed, edge)` 派生的确定性抖动 | `random_seed` 已设 |

另有一个**不走这个接口**的惩罚：转弯角度（`TurnPenalty`，见 [procedure-modeling](procedure-modeling.zh-CN.md)）。它是**路径相关**的——离开顶点 v 的代价取决于路径怎么进入 v——而 `Constraint::Evaluate` 只看单条边，拿不到入边信息。所以它被实现在 A\* 松弛循环内部（以及 Yen 的路径重定价里），而不是伪装成一个约束。这个边界值得记清：**约束层的表达力止于「单边函数」**，路径相关的规则必须进搜索循环。

## 5. 热路径纪律：字符串工作全部前置

约束在**每条边、每次搜索、每个 Yen spur** 上被求值。一次 k=10 的查询会跑上百次 spur 搜索，每次触及上万条边——所以约束的 `Evaluate` 里出现任何字符串比较、哈希、分配，都会被放大成灾难。

框架的纪律是：**所有按名字的匹配在构造期做完一次，热路径只做整数运算**。两个例子：

**`AvoidWaypointConstraint`** 收的是**已解析的顶点索引集合**，不是用户写的 ident 字符串。`ResolveAvoidVertices` 在查询开始时把 `"BOTON"` / `"BOTON/LF"` 解析成顶点号、排序去重，约束里只剩一次 `binary_search`。集合本身是排序 vector 而非 `unordered_set`：实际元素只有个位数，一次二分查找无需哈希、无大块分配，整个集合装得进一条 cache line。成员声明为 `const`，所以「已排序」这个不变量由类型系统保证，而非靠纪律维持。

**`AirwayRuleConstraint`** 更彻底，把匹配预计算成两张位掩码表：

```cpp
// 热路径全貌
const uint32_t m =
    (vertex_mask_[ctx.from] | vertex_mask_[ctx.edge.to]) & airway_mask_[ctx.edge.airway_id];
if (m == 0) {
  return EdgeVerdict::Allow();  // 绝大多数边走这里
}
if ((m & block_bits_) != 0) {
  return EdgeVerdict::Block();
}
double frac = 0.0;
for (uint32_t bits = m; bits != 0; bits &= bits - 1) {
  frac += fractions_[std::countr_zero(bits)];
}
return EdgeVerdict::Penalize(ctx.edge.distance_nm * frac);
```

第 i 位对应第 i 条规则，两表相与即「区域匹配且航路名匹配」。两次数组读加位运算，**比它取代的 `binary_search` 还便宜**。构造成本（仅规则非空时支付）：顶点表 V≈27.5 万 × 4B ≈ 1.1 MB、航路表 12k × 4B ≈ 48 KB，都是 per-query 栈生命周期，不常驻。

有个细节让构造也保持廉价：区域码在全库只有 **241 种**取值，而顶点有 27.5 万个。所以先把**每个不同的区域码**匹配一次进 241 项小表，再 O(V) 查表填充大数组——而不是对每个顶点重跑一遍前缀比较。

位掩码还带来一个结构性好处：**一条规则里列多少项都免费**。规则的区域侧和航路侧都是列表，同一条规则的所有条目共用**同一个位**，所以「中国大陆十个 FIR」和「一个区域」的区别只是 241 项小表里多几行被打上同一个 bit——热路径、内存、规则位占用全都不变。这个性质直接决定了 32 条规则的上限够用：`kMaxRules = 32` 限的是**规则条数**，不是条目数，一条 `kExact` 规则可以列两百个航路名。

## 6. 分层：约束不认识图构建器

`core/constraints/` **不依赖 `io/`**。这不是洁癖，而是有实际后果的：约束需要按名字匹配（区域码、航路 designator），而名字表在 `GraphBuilder` 里（`io/build/`）。

解法是把**解析器与约束分开**：解析器（`ResolveAvoidVertices` / `ResolveAirwayRules`）住在 `io/nav_database_routing.cc`，那里可以自由使用 `GraphBuilder`；约束只收解析产物（顶点索引数组、位掩码表）。所以约束类的构造函数签名里没有任何 `io/` 类型，`core/` 的编译不需要 `io/` 的头。

这也顺带解释了为什么解析阶段需要 `GraphBuilder::RegionOf(vertex)` 这个轻量访问器。原本只有 `IdentOf(vertex)`，它返回一个 owned `Ident`（materialize 出两个 `std::string`）——在 27.5 万次的循环里只为读一个字段而构造完整对象太重。`RegionOf` 直接返回紧凑 `FixedIdent` 里的 `string_view`，零构造。

## 7. 真正难的不是框架，是匹配粒度

框架本身很简单——接口三行、组合规则两条。issue #22（按国别过滤航路）的实现暴露出真正棘手的部分：**同一条规则，"匹配什么"有好几种合理读法，而它们的正确性差异巨大**。

需求本身很清楚：中国的 `J` 航路是终端过渡航路，不该出现在最终航路里；而美国的 `J` 航路（Jet routes）完全合法。所以规则形态是「区域 + 航路名前缀」。问题出在「封锁一条航路」到底封什么。

### 7.1 陷阱一：航路名不唯一

`route_identifier` 看起来像航路的唯一标识，**它不是**。实测 cycle 2601（按 ARINC 424 `waypoint_description_code` 第二列 `'E'`（End of Airway）切链）：

```
9858 个名字  →  15021 个物理实例   （平均 1.52 个/名）
2915 个名字（29.6%）有多个互不相连的实例；最多的一个有 18 个
```

而图层的 `airway_id` 是**按名字**去重的（同名 → 同 id）。所以「按 `airway_id` 集合封锁」实际是「按名字封锁全球」。对「区域 Z + 前缀 J」这条规则：

| 粒度 | 封锁 legs | 其中在中国境外 |
|---|---|---|
| **名字级**（= `airway_id` 集合） | 153 | **128** |
| 实例级（E-break 链 / 连通分量） | 25 | 0 |
| **逐段级**（leg 任一端点在区内） | 17 | 0 |

那 128 条境外误伤里，K2（美国）46 条、VA/VH（印度）24 条、K4 18 条、YB（澳洲）14 条……`J2` 一条名字横跨 ZM/MU/VA/K7/K4/K2 六个区域，`J22` 横跨七个。**名字级封锁会禁掉美国的合法 Jet routes，与需求正好相反。**

### 7.2 陷阱二：实例级也不够准

退一步用「实例级」（把物理航路实例还原出来，整条封）能消除跨国误伤，但仍会过度封锁。看「区域 Z + 前缀 J」命中的实例：

```
J103  DUMOL/VH ALDOM/VH ISBAN/VH ROBIN/VH SAPAX/VH → BEKOL/ZG   6 legs，仅末段触及 ZG
J104  SIKOU/ZG → RAGSO/VH DASON/VH COTON/VH CHALI/VH            4 legs，仅首段触及 ZG
J119  JHS/ZS → P37/ZS                                            1 leg，整条在区内
J22   VGA/ZM ODOKU/ZM TASET/ZM PAGVU/ZM TC/ZM                    4 legs，整条在 ZM
```

`J103` 是香港（VH）境内的一条 J 航路，末端伸进广州 FIR 一段。实例级会封掉整条 6 legs，**包括完全在香港境内的 5 legs**——那 5 legs 在香港合法，与「J 在中国是终端过渡」这条立法意图毫无关系。这是名字级同一种病的缩小版：误伤从 128 legs 降到 8 legs，性质没变。放大到全前缀规模，这类跨界连坐是 1275 legs。

实例级还有实现代价：图里没有「实例」概念，磁盘格式也不存（缓存格式刻意不改），得在内存里跑连通分量重建——而连通分量只能还原 **95.7%** 的真实链（4.3% 因航路分支被合并），等于引入一个新的近似误差层。

### 7.3 结论：逐段级

最终取**逐段级**：leg 的任一端点区域命中，即命中。三个维度全面占优——零跨国误伤、无需重建实例索引、热路径更便宜（位掩码 vs 多一层 per-edge → instance_id 间接，而 CSR 下没有 edge index 可用）。

需求提出者担心「截断会产生半条不可飞的航路」。这个顾虑在本项目的输出模型下不成立：航路输出是**点序列 + per-leg `via` 标签**（`MakeRoute` 逐 leg 取搜索实际遍历的边，`BuildRouteString` 按共享 designator 折叠成 filed-plan 形态）。某条航路的一段被封，A\* 只是不选它、绕开；不存在「飞到一半悬空」的输出形态。真正的连通性风险由默认 `penalize` 兜住，与粒度正交。

## 8. 陷阱三：前缀匹配对单条意图是危险的

规则的航路侧最初只做前缀匹配（「所有 J 航路」需要它）。但实测发现：**1371 个航路名是另一个航路名的严格前缀**（13610 对）。

| 探针 | 精确命中 | 纯前缀语义的额外误伤 |
|---|---|---|
| `J60` | 1 | **3**（J603 J604 J605） |
| `A3` | 1 | **52**（A30 A300 … A399） |
| `J2` | 1 | 35 |
| `W19` | 1 | 10（W190 … W199） |

「封锁 J60」这种最常见的单条意图，在纯前缀语义下会连带禁掉三条无关航路。而「所有 J 航路」这种类别意图又必须是前缀。**两者都是刚需**，所以规则带一个 per-rule 的 `Match{kExact, kPrefix}`，CLI 用尾随 `*` 显式区分（`J*` 前缀 / `J60` 精确）。

对称地问：区域侧要不要也分两种模式？**不要**。区域码最长 2 字符，所以 2 字符条目作为前缀已经等价于精确（没有更长的区域码能延伸它）。唯一的边界情形是单字符区域 `P`（1666 个航路点），前缀语义下会连带 PA/PB/…太平洋各区——窄到只需文档提一句「要精确就枚举 2 字符区域」，不值得为它引入第二个模式字段。

这里有个更一般的教训：**匹配语义的默认值应该看意图分布，而不是看哪个实现简单**。区域侧前缀就够，是因为数据形态（码长 ≤ 2）让两种语义收敛；航路侧不行，是因为数据形态（1371 个互为前缀）让两种语义分叉。同一个「前缀 vs 精确」的问题，在两侧的正确答案不同。

## 9. 收敛：语义唯一比向后兼容重要

有了精确模式后，旧的 `avoid_airways`（精确 designator → 硬封锁、全球生效）成了新规则的**真子集**：`{designators: {J60}, match: kExact, action: kBlock, regions: {}}`。

保留两套会让「封锁一条航路」有两种写法、两处实现、两处文档——而新形态**严格更强**（可以限定区域：「只禁中国的 J60，别处照用」是旧参数表达不了的）。所以 v3.23.0 把 `avoid_airways` 在三层全删（引擎字段、CLI `--avoid-awy`、JSON 字段），不留前端翻译层——翻译层本身就是第二个语义入口，与收敛目标矛盾。

`avoid_waypoints` **保留**：它的判据是**顶点身份**（「不经过 BOTON」），`AirwayRuleConstraint` 的判据是 region × designator，表达不了。相应地 `AvoidConstraint` 改名 `AvoidWaypointConstraint`（连头文件一起），让类名与职责一致——留一个名不副实的 `AvoidConstraint` 只会误导下一个读代码的人。

## 10. 一个约束管不到的地方：种子端点

`AvoidWaypointConstraint` 有个容易漏的补丁，值得单独说。它通过「封锁所有进入该顶点的边」来实现避让——但航路的**起终点连接 fix 是被「种下」的，不经由边进入**（见 [procedure-modeling](procedure-modeling.zh-CN.md) 的多源搜索）。所以一个被避让的顶点如果恰好是某个 SID 的出口 fix，约束抓不到它，它会作为搜索起点溜进结果。

修法在 `FindRoutes` 里：解析出的避让顶点集合**同时**用于从种子端点集合里剔除。这也是为什么 `ResolveAvoidVertices` 的返回值要排序去重——它被两处 `binary_search` 共用。

`AirwayRuleConstraint` 不需要对应的补丁：它匹配的是**边**而非顶点，种子 fix 本身不会被规则命中。但有个后果值得记：如果某机场的全部 SID 出口边都被 block，查询会返回「无航路」——这正是硬封锁不可逆的体现，属预期行为，不做特殊兜底。

## 11. 扩展一个新约束：清单

想加一条新规则（欧控 RAD/CDR、时段限制、RVSM 空域……），照这个清单走：

1. **判据是单边函数吗？** 是 → 约束层。否（路径相关，如转弯角）→ 进 A\* 松弛循环 + Yen 重定价，两处必须同步。
2. **硬还是软？** 会造成连通性断裂的规则优先做软——尤其是批量/区域级规则。硬过滤留给用户显式要求「禁止」的场合。
3. **软惩罚非负**，且优先做成**距离比例**而非固定量。固定 10 NM 对 30 NM 的短腿是重罚、对 300 NM 的长腿近乎无感；比例形式天然公平。默认强度参考 `0.5`（实测足以把目标航路挤出最优解，同时在无替代时仍可用）。
4. **按名字的匹配前置到构造期**，`Evaluate` 里只留整数运算。集合小就用排序 vector + `binary_search`；判据是「多维度组合」就用位掩码。
5. **解析器放 `io/` 层**（可以碰 `GraphBuilder`），约束只收解析产物——`core/` 不依赖 `io/`。
6. **`RouteRequest` 加结构化字段**，字符串语法留给 CLI/JSON 的反序列化层。引擎是库，CLI 只是它的一个前端。
7. **在 `FindRoutes` 里按需激活**：请求没设该字段 → 约束不进链，零成本。
8. **检查种子端点**：新约束的判据如果是顶点而非边，需要像避让那样额外剔除种子端点。
9. **不改缓存格式**（读侧特性无需 bump `format_version`），但仍要按版本纪律 bump 程序版本。
10. **测试**：单测覆盖匹配语义的每个分支（含边界与拒绝路径）；集成测试至少要有一条「不设该字段时路线逐字不变」的零回归硬线。

## 12. 小结

- **三态裁决**（Allow / Block / Penalize）一个接口装下硬禁止与软偏好；组合规则是「任一 block 则禁、软惩罚累加」，由搜索统一处理，约束彼此无知。
- **软惩罚保持 A\* 可采纳性**的前提是非负——想表达「偏好 A」要惩罚非 A，而非奖励 A。
- **热路径只做整数运算**：按名字的匹配全部前置到构造期，排序 vector（小集合）或位掩码（多维组合），并利用「不同取值远少于顶点数」把 O(V) 循环降成查表。
- **分层**：解析器在 `io/`（可用 `GraphBuilder`），约束在 `core/` 只收解析产物。
- **框架简单，粒度难**：issue #22 的教训是同一条规则「匹配什么」有名字级/实例级/逐段级三种读法，误伤差 7.5 倍（153 vs 17 legs），而最直白的读法恰好与需求相反。
- **匹配语义看数据形态**：区域侧前缀足够（码长 ≤ 2 使两种语义收敛），航路侧必须分精确/前缀（1371 个名字互为前缀使两种语义分叉）。
- **语义唯一优先于向后兼容**：新能力覆盖旧参数时收敛掉旧的，不留翻译层。
