# 跨平台二进制缓存：.bfdb / nav_cifp.bfdb

> 把 ~1.5s 的"解析 + 建图"变成 ~50ms 的"读文件"，且文件在 x86/ARM 之间可移植、单文件
> 部署。面向想理解缓存格式取舍的读者。相关代码：`io/cache/`（`byte_io.h`、`graph_cache.*`、
> `graph_snapshot.h`、`cifp_cache.*`）、`io/graph_builder.cc`（`FromSnapshot`/`ToSnapshot`）。

## 1. 问题：每次启动都要重新解析建图

X-Plane 数据全量解析 + 建图有成本（解析 ARINC 424 尤甚）。冷启动 release 实测 ~1.56s、
debug ~7.7s。对一个"快速出结果"的 CLI，这个启动开销不可接受——尤其你只想查一条航路。

方案：`bf build` 把建好的图 + 元数据序列化成紧凑二进制 `.bfdb`，`bf route --db` 直接反序列化
跳过全部解析。实测 **release 1.56s → 0.059s（~26×）**，debug 7.7s → 0.91s，两条路径产出逐字节
相同。文件 ~18MB。

## 2. 核心取舍：显式定宽小端，而不是 mmap

早期设想过 mmap（零拷贝、直接把文件映射成内存结构）。**M4 推翻了这个方向**，原因是它与
"跨平台可移植"直接冲突：

- mmap 零拷贝要求**磁盘布局 = 内存布局**，会把 struct padding、字节序、`sizeof` 焊死进文件；
- x86 和 ARM 的布局/对齐可能不同，一个平台产出的文件另一个平台读不了；
- 而单进程 CLI 场景，mmap 省下的那点 memcpy（~10ms）相对建图成本可忽略。

用户明确"可移植优先、mmap 不重要"，所以改为**显式逐字段序列化**：

- 整数一律**定宽小端**（LSB first），与主机字节序无关；
- 浮点写 **IEEE-754 位模式**（`memcpy` 到同宽无符号整数），当前所有平台共享，精确往返；
- 读时逐字段解析重建进 `std::vector`，**不 dump struct**。

任何平台（x86/ARM）产出与读取一致。工具是 `byte_io.h` 的 `ByteWriter`/`ByteReader`
（带边界检查，读越界置错误标志、优雅降级）。

## 3. 磁盘按顶点 record，内存按 struct-of-arrays

内存里图是 **struct-of-arrays**（SoA）：`coords_` / `idents_` / `on_network_` / `kinds_` 各一条
平行数组。这是为 A\* 热路径的 cache 友好——搜索只碰 `coords` 和 `edges`，不碰 ident 字符串，
把冷热数据分开就不会把没用的 ident 拉进 cache 行。

但**磁盘格式不必跟随内存布局**——它只在 `Open` 时顺序读一遍、再分发填回各 SoA 数组，磁盘上
是 AoS 还是 SoA 对运行时零影响。早期版本盲目照抄内存 SoA，把每个属性写成一段独立的平行数组,
于是"加一个 per-vertex 字段"= 多一条平行数组 + `Build`/`Open` 各加一段 + 手工维护 `size==V`
不变量，越加越散。这正是第 2 节"mmap 焊死磁盘=内存布局"的反面教训在自研格式里的翻版。

v3 起磁盘改为**逐顶点一条自包含 record**（`coord + ident 引用 + flags + kind`）：加一个
per-vertex 字段就是 record 里多一个字段，没有新平行数组、没有 size 不变量。**机场专属字段**
（如 elevation）单独放一个**机场 record 段**（只 `[first_airport_vertex, V)` 的 ~1.5 万条），
不摊到 25 万顶点上——语义与布局对齐，机场字段各归其位。

刻意不上 TLV/段目录：`.bfdb` 是本地 `bf build` 产物、非跨版本分发，加字段时 bump
`format_version` + 重建缓存即可，跨版本兼容的价值不足以抵消段目录的复杂度。

## 4. 与 protobuf 的异同

同样是"逐字段、不 dump struct、浮点 IEEE754 小端"。但我们刻意不同：

- **定宽而非 varint**：我们的 offsets/顶点索引普遍是大整数，定宽更快、且能配合扩容前置分配；
- **无 per-field tag**：`.bfdb` 是单一生产者 + 单一消费者的本地私有缓存，格式演进靠文件头的
  `format_version` 整体版本号，不需要字段级前后兼容；
- **不引入 protobuf 依赖**（违反项目极简依赖原则）。

## 5. 哪些不落盘：三个 lookup map

图里有三个 `unordered_map`（`ident_index_`/`ident_first_`/`airport_index_`）做名字→顶点查找。
它们**不序列化**，加载后从 `idents_` 重建（`GraphBuilder::RebuildIndices`）：

- `unordered_map`（哈希桶 + 堆节点指针）结构上不可移植序列化；
- 实测它们运行时占 ~30MB > `.bfdb` 文件本身（~17MB），落盘是纯亏；
- 重建成本 reserve(V) 后 ~50–100ms，一次性、冻结只读。

`FromImage`/`ToImage`（`io/graph_builder.cc`）是图与缓存镜像 `BfdbImage` 之间的转换点。

## 6. 字符串：文件层用池引用，运行时保持拥有型

曾考虑把 idents/airway 名全改成 `string_view` + 集中字符串池省内存。**实测否决**：cycle 2601
数据里 ident 最长 5 字符、region 1–2、airway 名 99%+ ≤10 字符，**全部落在 libc++ SSO（22B）
阈值内 → 本就零堆分配**。view 化的主收益（消堆分配）不存在，代价却是贯穿全 API 的 lifetime
契约，还逆了"领域类型是不可变值类型"的设计宪法。

结论：**运行时保持拥有型 `Ident`（全 SSO，无碎片）；只在序列化文件层**用 `{u32 offset, u32 len}`
引用字符串池令文件紧凑，加载后重建回拥有型。鱼与熊掌部分兼得、零 lifetime 风险、零 API 改动。
`StringPool` 后来加了**去重**（`unordered_map` 记已 intern 的串）：ident/region/airway/ICAO
高度重复，同串只存一份。去重对 reader **透明**——引用格式 `{offset,len}` 与段布局都没变，旧
reader 读新文件、新旧文件互读皆可，故**不 bump format_version**。实测（真实 2601 数据）graph
-4.7%、CIFP -10.6%、detail -9.8%。

## 7. GraphEdge 瘦身到 16B

边数组是图里最大的结构、A* 热路径逐边遍历。`GraphEdge` 从实现期偏离的 32B 回归设计意图的
**16B**：

```
int32  to           // 目标顶点
float  distance_nm  // 存 float；A* 的 g 值/路径长用 double 累加，精度无损
uint16 airway_id    // 唯一 airway 名 ~12k << 65535，建表加 >65535 保险丝
int16  base_fl, top_fl
uint8  flags        // bit0=is_high，余位 RAD/CDR 预留
```

边数组体积腰斩，A* 遍历时一条 cache 行能装的边数翻倍。注意磁盘上 GraphEdge 是 **15B**
（4+4+2+2+2+1，无内存对齐 padding），比内存 16B 更紧。

## 8. CIFP 分段缓存：单文件部署 + 按需加载

图缓存不含程序数据（CIFP 按需从 `CIFP/*.dat` 解析）。为让部署只需缓存文件、无需带 14838 个
CIFP 散文件，另有一个**分段索引**缓存 `nav_cifp.bfdb`（magic "BFCP"）：

- 结构：header + `ICAO→(段偏移, 段长)` 目录 + 每机场一段自包含的 `CifpData`（含段局部字符串池，
  可独立反序列化）；
- **按需加载**（默认 `on-demand`）：`Open` 只读 header + 目录进内存（~1.5MB），`Fetch(icao)` 才
  按段偏移**定位读**（pread / Windows `ReadFile`+`OVERLAPPED`）该段——启动仍毫秒级，不常驻全部程序；
- **eager 模式**：`Open` 时 `FetchAll` 全量反序列化进内存并冻结（~102MB），之后无锁读，面向
  Web/批量并发。

实测 CIFP 缓存 ~44MB / 14838 机场（结构化后远小于 105MB 原始文本），`--data` 指空目录仍能
从缓存出全 SID/STAR、与文件路径逐字节一致。

> `Fetch` 在 `Open` 时开的**一个只读句柄**上做定位读（pread / `ReadFile`+`OVERLAPPED`，都按显式
> 偏移读、不动共享文件位置）→ 并发查异机场天然无锁安全，见
> [thread-safety.zh-CN.md](thread-safety.zh-CN.md)。（早期每次 `Fetch` 都重开 `ifstream`，
> 现改为共享句柄免去 per-fetch open 开销。）

## 9. 三层版本体系

缓存格式会演进，必须能干净拒绝不兼容的旧文件而非崩溃。三层版本：

1. **程序 semver**（CMake `project VERSION` → `core/version.h` 的 `kBravoFinderVersion` →
   `bf --version`）；
2. **每类缓存 `format_version`**（图 "BFDB"、CIFP "BFCP"），机器校验，不符走
   `Result::Err(kDataMissing)`，提示重跑 `bf build`；
3. **provenance**：程序 semver + source_loader + AIRAC cycle/build 写进缓存 header。

纪律：**改缓存磁盘布局 → bump 对应 `format_version`**；仅读取侧/内部函数改动不动布局，不 bump。

## 10. 健壮性：损坏文件优雅报错，不崩溃

反序列化面对的是可能损坏/截断/伪造的文件。所有从文件头读出的**计数字段**（顶点/边/airway/
段数、串长）在 `resize` 之前都用"剩余字节 ÷ 每元素最小磁盘字节数"设上界，越界即走
`Result::Err(kDataMissing)`，绝不因 `bad_alloc`/`length_error` 崩溃。段偏移/段长在打开目录时
与文件大小交叉校验。这条"损坏走 Result 而非崩溃"是格式的正确性契约。

## 11. 小结

- **可移植 > 零拷贝**：放弃 mmap，改显式定宽小端 + IEEE754 位模式，x86/ARM 通用；
- **磁盘按顶点 record、内存按 SoA**：加 per-vertex 字段=record 加一行，机场专属字段单独分段；
- **不落盘 lookup map**（重建更省）、**文件层字符串池**（运行时仍拥有型全 SSO）；
- **图缓存 + CIFP 分段缓存**两文件，按需加载单机场段，单文件部署；
- **三层版本 + 计数上界校验**，格式演进可控、损坏优雅报错；
- 净效果：启动 ~26×，18MB 图 + 44MB 程序，跨平台一致。
