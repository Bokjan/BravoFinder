# 航路串压缩：ICAO filed-flight-plan 与并线航路

> 为什么 BravoFinder 输出的航路串不是「逐点全列」，而是像真实飞行计划那样只在进出点列航路名—— 以及处理并线航路(concurrent airways)时一个不折叠就会出假转接点的陷阱。面向想理解航路串 输出格式的读者。相关代码：`lib/core/routing/route_string.{h,cc}`、`lib/core/routing/route.h`。

## 1. 两种航路串：全列 vs 压缩

一条航路在内部是**逐段**表示的：每个 `RouteLeg` 有 `to`（到达点）和 `via`（经由的航路）。最朴素的 输出是把每一段都列出来：

```
KLAX GARDY V210 HESPE V210 APLES V442 HEC J64 GLACO J64 PGS J64 ...
```

但真实的 ICAO 飞行计划（Doc 4444 格式）**不这么写**。同一条航路上的连续过路点会被省略，只保留 「上这条航路的点」和「下这条航路的点」：

```
KLAX SID GARDY V210 APLES V442 HEC J64 RSK J110 ALS ...
```

`V210` 从 GARDY 一路飞到 APLES，中间的 HESPE 不列。这就是 `BuildRouteString` 做的**折叠**：把 `via` 相同的连续 leg 合并成一段，只在边界列航路名。开头的 `SID`（结尾的 `STAR`）是**字面量连接词**， 不是程序名——具体的 SID/STAR 名字放在 `Route::sid`/`star` 字段里（详见 §5）。

## 2. 陷阱：不能按 `via` 字符串相等来折叠

朴素的折叠会写成「连续 leg 的 `via` 字符串相等就合并」。这在**并线航路**上会出错。

X-Plane 的 `earth_awy.dat` 把物理上重合的多条航路编码成一个连字符串，如 `V28-Y28`——意思是 这一段**同时是** V28 和 Y28。于是一条实际全程在 Y28 上的航路，数据里可能长这样：

```
… Y28 … V28-Y28 … Y28 …
```

按字符串相等折叠，会把它看成 `Y28`→`V28-Y28`→`Y28` **三段不同的 via**，输出成「中途换了两次 航路」——凭空造出两个假转接点。真实情况是：全程都能算在 Y28 上，不该断开。

## 3. 正解：对 designator 列表求累积交集

关键洞察：每段 `via` 不是一个航路名，而是一个**航路名集合**(`V28-Y28` = {V28, Y28})。连续几段 能否折叠，取决于它们的集合有没有**共同 survivor**——即累积交集是否非空。

算法(`BuildRouteString`):

1. 把当前 leg 的 `via` 按 `-` 拆成 designator 列表，作为初始 `running` 集合。
2. 向后看下一段：求 `running` 与它的列表的**交集**。非空 → 继续吞并、`running` 收窄为交集； 空 → 真正换航路，断开。
3. 一组折叠完，选定的航路名 = `running` 的**第一个 survivor**。

```cpp
std::vector<std::string> running = SplitDesignators(legs[i].via);
size_t j = i;
while (j + 1 < legs.size() && legs[j + 1].via != "DCT") {
  std::vector<std::string> inter = Intersect(running, SplitDesignators(legs[j + 1].via));
  if (inter.empty()) break;        // 交集空 → 真正换航路
  running = std::move(inter);      // 收窄,继续
  ++j;
}
const std::string& chosen = running.front();
```

回到 §2 的例子：`Y28`∩`V28-Y28` = {Y28} 非空，`{Y28}`∩`Y28` = {Y28} 非空 → 三段折叠成一段 `Y28`，假转接点消失。

## 4. 为什么「第一个 survivor」是对的

选交集的第一个元素当输出航路名，合法性来自一个包含关系：

> chosen ∈ 累积交集 ⊆ 每一段各自的 designator 列表

即 `chosen` 在这一组的**每一段**上都真实存在，写它到任何一段都不撒谎。至于选哪个 survivor—— 交集里的元素**没有航空语义上的优劣**，选第一个只需保证**确定性**（源数据出现顺序即可）。项目 owner 明确不要求特定排序，所以不引入无谓的排序规则。

数据里并线可多达 10 条(`A203-A336-…`);也有「整段都不收敛到单一名」的真实例子： `DALTI→KABRA→PARLO` 全程 `G325-J133`，交集始终是 {G325, J133}，选 `G325`。

## 5. 边界：DCT / SID / STAR 不折叠

`DCT`（直飞段）是**硬边界**，永不参与折叠——它单独成段输出，`concurrent_airways` 清空：

```cpp
if (legs[i].via == "DCT") {
  legs[i].concurrent_airways.clear();
  rs += " DCT " + legs[i].to;
  ++i;
  continue;
}
```

机场到航路网的衔接腿（离场 SID、进场 STAR）在 `via` 里存的是**字面量关键字** `"SID"`/`"STAR"`， 而不是程序名（`OPPAR4`、`ABEY2G` 这类名字放在 `Route::sid`/`star` 字段）——航路串因此保持 航空/ICAO 风格：`RJTT SID JYOGA … ABBEY STAR VHHH`。这两个 token 与任何航路 designator 都不相交， 折叠的累积交集在它们处必然清空，所以无需像 `DCT` 那样特判就天然单独成段。这样一来 `ParseRoute` 把计算出的航路串喂回时，也靠这两个关键字（而非查程序库）复原衔接腿，round-trip 稳定。

## 6. 并线信息不丢：concurrent_airways 字段

折叠「选一个名」是为了航路串简洁，但**并线这个事实本身不该丢**——它是有用的运行信息。所以每个 被折叠的 leg 除了 `via`（单一 designator）外，还填一个 `concurrent_airways` 字段：只有真正并线的段 （原始列表 > 1）才非空。

- JSON 输出：仅并线段带 `concurrent_airways` key。
- 文本表：via 列附注 `(concurrent: V210, V394)`。

## 7. 坐标与累计距离：points[] 与 cumulative_nm

路由 JSON 除航路串与分段外，还直出两组机器可读数据，消费方（CLI-json / MCP / HTTP 服务）无需再自行推导：

- **`points[]{ident,lat,lon}`**：沿航路的有序点，各带 ident 与坐标（十进制度，WGS-84）。`points` 与 `legs` 平行但多一项——**N 个点、N-1 条 leg**，leg i 的目的点是 `points[i+1]`。坐标取自 `Route::points[].coord`， 过去只有文本表打印、JSON 缺失，现补齐。
- **每条 leg 的 `cumulative_nm`**：到该 leg 为止的累计距离（`route_metrics.h::CumulativeDistances`，与 `legs` 平行），末项等于 `total_distance_nm`。

坐标/距离小数位沿用序列化器的 `SetMaxDecimalPlaces`（route 输出为 2 位）。这些字段与 `route_json.h` 共享， 一处改三端受益，不涉及 cache 层、不 bump `format_version`。

## 8. 实现取舍：纯函数 + vector 交集

- **纯函数**:`BuildRouteString(first_point, legs)` 原地改写每个 leg 的 `via` 并填 `concurrent_airways`，无副作用、易测。属 `bf_core`。
- **vector 而非 set 求交集**：每组并线最多 ~10 条，线性 `Intersect` 比 `std::set` 的树结构更轻 （`SplitDesignators` 用 `-` 拆分——真实 ATS 航路名是「字母+数字」无内部连字符，`-` 是无歧义分隔符）。
- **JSON 作库 API**：序列化在 header-only 模板 `bf::WriteRouteJson`(`route_json.h`)里，靠 RapidJSON 风格 Writer 的鸭子接口，库用户自带 rapidjson 即可用，`bravofinder` 零新增依赖。

单测覆盖折叠全场景(`route_string_test.cc`)与零依赖 JSON API(`route_json_test.cc` 用不链 rapidjson 的 StubWriter 驱动)。
