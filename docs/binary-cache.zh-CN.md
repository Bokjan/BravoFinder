# 跨平台二进制缓存：统一 。bfdb

> 把 ~1.5s 的「解析 + 建图」变成 ~50ms 的「读文件」，且文件在 x86/ARM 之间可移植、单文件 部署。面向想理解缓存格式取舍的读者。相关代码：`lib/io/cache/`（`byte_io.h`、`unified_cache.*` 容器 + `graph_codec.*`／`cifp_codec.*`／`nav_detail_codec.*` 三段 codec、`graph_snapshot.h`）、 `lib/io/graph_builder.cc`（`FromSnapshot`/`ToSnapshot`）。

## 1. 问题：每次启动都要重新解析建图

X-Plane 数据全量解析 + 建图有成本（解析 ARINC 424 尤甚）。冷启动 release 实测 ~2.27s、 debug ~7.6s。对一个「快速出结果」的 CLI，这个启动开销不可接受——尤其你只想查一条航路。

方案：`bf build` 把建好的图 + 程序 + 导航台细节序列化成**一个**紧凑二进制 `.bfdb`， `bf route --db` 直接反序列化跳过全部解析。实测 **release 2.27s → 0.20s（~11×）**， debug 7.6s → 1.4s，两条路径产出逐字节相同。单文件 60.9MB（含 graph + CIFP + detail， 全局 pool 去重后比三文件分离省 ~8MB）。磁盘/加载/运行内存的完整对比见第 8 节末的表。

## 2. 核心取舍：显式定宽小端，而不是 mmap

早期设想过 mmap（零拷贝、直接把文件映射成内存结构）。**M4 推翻了这个方向**，原因是它与 「跨平台可移植」直接冲突：

- mmap 零拷贝要求**磁盘布局 = 内存布局**，会把 struct padding、字节序、`sizeof` 焊死进文件；
- x86 和 ARM 的布局/对齐可能不同，一个平台产出的文件另一个平台读不了；
- 而单进程 CLI 场景，mmap 省下的那点 memcpy（~10ms）相对建图成本可忽略。

用户明确「可移植优先、mmap 不重要」，所以改为**显式逐字段序列化**：

- 整数一律**定宽小端**（LSB first），与主机字节序无关；
- 浮点写 **IEEE-754 位模式**（`memcpy` 到同宽无符号整数），当前所有平台共享，精确往返；
- 读时逐字段解析重建进 `std::vector`，**不 dump struct**。

任何平台（x86/ARM）产出与读取一致。工具是 `byte_io.h` 的 `ByteWriter`/`ByteReader` （带边界检查，读越界置错误标志、优雅降级）。

## 3. 磁盘按顶点 record，内存按 struct-of-arrays

内存里图是 **struct-of-arrays**（SoA）：`coords_` / `idents_` / `has_outbound_` / `has_inbound_` / `kinds_` 各一条平行数组。这是为 A\* 热路径的 cache 友好——搜索只碰 `coords` 和 `edges`，不碰 ident 字符串，把冷热数据分开就不会把没用的 ident 拉进 cache 行。

但**磁盘格式不必跟随内存布局**——它只在 `Open` 时顺序读一遍、再分发填回各 SoA 数组，磁盘上 是 AoS 还是 SoA 对运行时零影响。早期版本盲目照抄内存 SoA，把每个属性写成一段独立的平行数组， 于是「加一个 per-vertex 字段」= 多一条平行数组 + `Build`/`Open` 各加一段 + 手工维护 `size==V` 不变量，越加越散。这正是第 2 节「mmap 焊死磁盘=内存布局」的反面教训在自研格式里的翻版。

v3 起磁盘改为**逐顶点一条自包含 record**（`coord + ident 引用 + flags + kind`）：加一个 per-vertex 字段就是 record 里多一个字段，没有新平行数组、没有 size 不变量。**机场专属字段** （如 elevation）单独放一个**机场 record 段**（只 `[first_airport_vertex, V)` 的 ~1.5 万条）， 不摊到 25 万顶点上——语义与布局对齐，机场字段各归其位。

刻意不上 TLV/字段级段目录：`.bfdb` 是本地 `bf build` 产物、非跨版本分发，加字段时 bump 容器 `format_version` + 重建缓存即可，跨版本兼容的价值不足以抵消其复杂度。（最近一次即 `format_version` 5→6：CIFP 每条 leg 增补 RNP / 转向 / 速度限制三字段，旧缓存被拒、`bf build` 重建。容器层确有一个**固定 3 项的段表**定位 graph/cifp/detail 三段，见第 8 节——那是分段容器 的必需骨架，不是可扩展的字段级 TLV。）

## 4. 与 protobuf 的异同

同样是「逐字段、不 dump struct、浮点 IEEE754 小端」。但我们刻意不同：

- **定宽而非 varint**：我们的 offsets/顶点索引普遍是大整数，定宽更快、且能配合扩容前置分配；
- **无 per-field tag**：`.bfdb` 是单一生产者 + 单一消费者的本地私有缓存，格式演进靠文件头的 `format_version` 整体版本号，不需要字段级前后兼容；
- **不引入 protobuf 依赖**（违反项目极简依赖原则）。

## 5. 哪些不落盘：三个 lookup 索引

图里有三个名字→顶点查找索引（`ident_index_`/`ident_all_`/`airport_index_`）。它们**不序列化**， 加载后从 `idents_` 重建（`GraphBuilder::RebuildIndices`）：

- 索引结构（早先是 `unordered_map`，2026-07 起是**排序数组 + 二分**）不落盘更省——落盘要么不可移植 （哈希表的桶/指针），要么就是把能 O(n log n) 重建的东西白占文件；
- 重建成本：填充后 `std::sort`，一次性、之后冻结只读。

> **为什么从 `unordered_map` 换成排序数组**：一个隔离微基准（真实 27 万 idents）量出查找只慢 ~49ns/op、而内存从 ~26MB 降到 ~4MB，且查找不在 A\* 热路径（只在端点解析）——拿确定的内存收益换 不可感知的延迟。key 用定长 `FixedIdent`(12B) / `FixedIdentNoRegion`(8B，无 region 的 ICAO/bare ident)，`pair<key,int>` 因此保持 12–16B。`RebuildIndices` 末尾有 `is_sorted` 断言，防「未排序 → 二分静默错」。

`FromSnapshot`/`ToSnapshot`（`lib/io/graph_builder.cc`）是图与缓存快照 `GraphSnapshot` 之间的转换点。

## 6. 字符串：文件层用池引用，运行时按体量分层

曾考虑把 idents/airway 名全改成 `string_view` + 集中字符串池省内存。**实测否决**：cycle 2601 数据里 ident 最长 5 字符、region 1–2、airway 名 99%+ ≤10 字符，**全部落在 libc++ SSO（22B） 阈值内 → 本就零堆分配**。view 化的主收益（消堆分配）不存在，代价却是贯穿全 API 的 lifetime 契约，还逆了「领域类型是不可变值类型」的设计宪法。

结论：**只在序列化文件层**用 `{u32 offset, u32 len}` 引用字符串池令文件紧凑，加载后重建回**拥有型** 值——零 lifetime 风险。运行时的拥有型本身按体量分层：查询边缘的少量字符串用全 SSO 的 `Ident`；而 V 级（27 万顶点 ident）与 CIFP 级（76 万 leg 的 fix）用定长 12B 的 `FixedIdent`（length-prefixed inline，零堆分配）——仍是拥有型、零 lifetime 风险，只是把「弹性」换成「紧凑」，各省约 14MB / 40MB。 `StringPool` 带**去重**（`unordered_map` 记已 intern 的串）：ident/region/airway/ICAO 高度重复，同串只存一份。去重对 reader **透明**——引用格式 `{offset,len}` 与段布局都没变。

统一容器进一步把去重推到**全局**：graph、CIFP、detail 三段共用**一个** `StringPool`， 所以一个 fix 名即便在图顶点、几十个机场的程序腿、导航台细节里都出现，全库也只存一份。 过去 CIFP 每个 segment 各带一份局部池是重复的放大器（同名在几十段里各存一次）；统一后 segment 去掉局部池、引用直指全局池。实测 cycle 2601：三段池之和 ~8.7MB → 全局去重后 ~1.5MB（pool 体积 −83%），是单文件比三文件省 ~8MB 的主因。

## 7. GraphEdge 瘦身到 16B

边数组是图里最大的结构、A* 热路径逐边遍历。`GraphEdge` 从实现期偏离的 32B 回归设计意图的 **16B**：

```
int32  to           // 目标顶点
float  distance_nm  // 存 float；A* 的 g 值/路径长用 double 累加，精度无损
uint16 airway_id    // 唯一 airway 名 ~12k << 65535，建表加 >65535 保险丝
int16  base_fl, top_fl
uint8  flags        // bit0=is_high，余位 RAD/CDR 预留
```

边数组体积腰斩，A* 遍历时一条 cache 行能装的边数翻倍。注意磁盘上 GraphEdge 是 **15B** （4+4+2+2+2+1，无内存对齐 padding），比内存 16B 更紧。

## 8. 统一容器：一个文件装三段 + CIFP 按需加载

早期是**三个文件**（图 `nav_*.bfdb`、程序 `*_cifp.bfdb`、细节 `*_detail.bfdb`），靠文件名 派生关联。问题：部署要管三个文件、文件名派生是隐式契约、三段可能来自不同 AIRAC cycle 而错配、 且 CIFP 每段局部池导致字符串大量重复。现在合并为**一个统一 `.bfdb`**（magic 仍 「BFDB」）：

```
[file header]   magic "BFDB", format_version, section_count, cycle,
                program_version, source_loader, data_dir, pool_len
[section table] 固定 3 项，每项 (type U32, offset U64, length U64)；
                offset==length==0 表示该段缺席
[global pool]   pool_len 字节，三段共用（见第 6 节）
[graph 段] [cifp 段] [detail 段]   顺序紧接，偏移由段表指定
```

- **pool 放头部而非尾部**：Writer 本就要 buffer 各段体（段表需要每段 offset/length），pool 在 序列化中自然累积完成，写盘顺序 header→段表→pool→各段体，无额外成本；Reader 顺序读完 header+段表+pool（~1.5MB）后 pool 已就位，解析各段即时 resolve，不需 seek 到文件尾、不需 全量读入。对 CIFP 按需尤其友好：读完 pool 与目录即可开始 `Fetch`，graph/detail 段可跳过。
- **CIFP 段**：`airport_count` + `ICAO→(段内相对偏移, 段长)` 目录 + 每机场一段 bare body （无局部池，引用直指全局池）。段内相对偏移加上 CIFP 段的 `section_offset` 得绝对文件偏移。
- **按需加载**（默认 `on-demand`）：`Open` 只读 header+段表+pool+CIFP 目录进内存（~3MB）， `Fetch(icao)` 才按绝对偏移**定位读**（pread / Windows `ReadFile`+`OVERLAPPED`）该段——启动 仍毫秒级，不常驻全部程序；
- **eager 模式**：`Open` 时 `FetchAll` 全量反序列化进内存并冻结（~102MB），之后无锁读，面向 Web/批量并发。
- **CIFP 段强制写入**：无程序的缓存无法解析 SID/STAR，故 CIFP 段始终写入、段表项 offset/length 非零；airport 程序由该段提供，缺席即报告无程序。

`--data` 指空目录仍能从缓存出全 SID/STAR、与文件路径逐字节一致。

> `Fetch` 在 `Open` 时开的**一个只读句柄**上做定位读（pread / `ReadFile`+`OVERLAPPED`，都按显式 偏移读、不动共享文件位置）→ 并发查异机场天然无锁安全，见 [thread-safety.zh-CN.md](thread-safety.zh-CN.md)。全局 pool 是 `CifpArchive` 持有的只读 `std::string`，`Fetch` 只读它 resolve 引用，同样无共享可变状态。

### 磁盘 / 加载速度 / 运行内存对比

实测 cycle 2601、release 构建、单条 `KJFK→KLAX` 查询、峰值 RSS 由 `/usr/bin/time -v` 采集 （32 核工作站）：

| 场景 | 磁盘 | 冷启动加载 | 峰值 RSS |
|---|---:|---:|---:|
| 原始 `.dat` 解析建图（`--data`） | — | ~2.27s | ~165 MB |
| 统一 `.bfdb`，on-demand（`--db`，默认） | 60.9 MB | ~0.20s | ~101 MB |
| 统一 `.bfdb`，eager（`--cifp-load eager`） | 60.9 MB | ~0.29s | ~169 MB |

读要点：

- **磁盘**：统一 60.9 MB vs 旧三文件分离（图 ~18 + CIFP ~45 + detail ~2.5 ≈ 65.5 MB）， 全局池去重省 ~8MB（三段局部池之和 8.7MB → 全局 1.5MB，见第 6 节）。
- **加载速度**：缓存路径 ~0.20s vs 原始解析 ~2.27s，**~11×**。on-demand 与 eager 加载耗时相近 （都只在 `Open` 读 header+段表+pool+CIFP 目录 ~3MB），差别在 eager 额外 `FetchAll` 反序列化 全部程序。
- **运行内存**：on-demand ~101 MB（图 + MORA/MSA + detail + ~1.5MB 全局池 + ~1.5MB CIFP 目录， 程序段按需拉）；eager ~169 MB（+~67MB 全部程序常驻，换无锁读）；原始解析 ~165MB（解析中间 态更吃内存）。选型：一次性 CLI 查询用 on-demand（内存最省、启动最快）；Web/批量并发用 eager （常驻程序、无锁读，见第 8 节）。

## 9. 职责分层：容器管文件，codec 管字节

统一后有清晰的两层：

- **容器 `UnifiedCache`**：唯一做文件 I/O 的一层，拥有文件头、段表、全局池、以及**唯一**的容器级 `format_version`。`Build` 编排三段 codec 写盘，`Open` 读回并分发，`ReadHeader` 只读容器头 （cycle + provenance）供 `BfdbInventory` 廉价编目。
- **三段 codec `GraphCodec`/`CifpCodec`/`NavDetailCodec`**：只管「结构体 ↔ bytes」，不碰文件、不写 文件头、无自己的版本号。`Encode(…, ByteWriter&, StringPool&)` 把段体追加进容器给的 writer、 把串 intern 进容器给的全局池；`Decode(bytes, pool)` 反之。

这也解释了 `GraphSnapshot` 为何只剩纯图数据：cycle/provenance/data_dir 全部上移容器头，只存一份， 不再每段重复。（旧 `graph_cache.*`/`cifp_cache.*`/`nav_detail_cache.*` 已改名 `*_codec.*`—— `_cache` 暗示持久化/文件读写，语义已归容器；`_codec` 精确描述新职责。）

## 10. 三层版本体系

缓存格式会演进，必须能干净拒绝不兼容的旧文件而非崩溃。三层版本：

1. **程序 version**（CMake `project VERSION` → `lib/core/version.h` 的 `kBravoFinderVersion` → `bf --version`）；
2. **容器 `format_version`**（magic 「BFDB」，当前 = 4），机器校验，不符走 `Result::Err(kFormatMismatch)`，提示重跑 `bf build`。**只此一个版本号**管全部布局—— 统一容器一次性产出三段，不存在「只改 graph 段但 CIFP 段保持旧版」的场景，故不设 per-section 版本号（那是过度设计）；
3. **provenance**：程序 version + source_loader + AIRAC cycle 写进容器头。

> 注：`build`（X-Plane `.dat` 头行的 `build YYYYMMDD`）已从格式中删除——它是 X-Plane 专有字段， Navigraph 通用元数据（`cycle_info.txt`/`cycle.json`）只有 `cycle` 和 `revision`，其余格式 （iFMS、Little Navmap、CustomData）均无 `build`。统一格式只保留 `cycle` 作主键，`cycle=0` 表示无 AIRAC 出处。

纪律：**改缓存磁盘布局 → bump 容器 `format_version`**；仅读取侧/内部函数改动不动布局，不 bump。

## 11. 健壮性：损坏文件优雅报错，不崩溃

反序列化面对的是可能损坏/截断/伪造的文件。所有从文件头读出的**计数字段**（顶点/边/airway/ 段数、串长）在 `resize` 之前都用「剩余字节 ÷ 每元素最小磁盘字节数」设上界，越界即走 `Result::Err`，绝不因 `bad_alloc`/`length_error` 崩溃。段表里每个存在的段偏移/段长在 `Open` 时与文件大小交叉校验；CIFP 段内相对偏移与 CIFP 段长交叉校验。这条「损坏走 Result 而非崩溃」 是格式的正确性契约。

## 12. 小结

- **可移植 > 零拷贝**：放弃 mmap，改显式定宽小端 + IEEE754 位模式，x86/ARM 通用；
- **磁盘按顶点 record、内存按 SoA**：加 per-vertex 字段=record 加一行，机场专属字段单独分段；
- **不落盘 lookup map**（重建更省）、**文件层字符串池**（运行时仍拥有型全 SSO）；
- **一个统一 `.bfdb`**：graph + CIFP + detail 三段共用全局池、单文件部署，CIFP 段按需定位读；
- **容器管文件 / codec 管字节**两层分工，单一容器 `format_version` + 计数上界校验；
- 净效果：启动 ~11×（release 2.27s→0.20s），单文件 60.9MB（全局池去重省 ~8MB），跨平台一致。
