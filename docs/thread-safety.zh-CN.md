# 线程安全契约 B：一个数据库，多线程并发查询

> 从一开始就立的设计约束，不是事后补丁。面向想在 Web / 批量场景并发调用 BravoFinder 的 读者。相关代码：`lib/io/nav_database.{h,cc}`、`lib/io/cache/cifp_codec.*`、`lib/core/graph/`。

## 1. 契约内容

**契约 B**：`NavDatabase::Open()`（或 `OpenCached()`）成功后，实例除一个**内部同步的程序 缓存**外完全只读；`FindRoutes()` / `MsaForAirport()` 是 `const`，**可被多线程并发调用于同一 实例**。面向未来的 Web API / 批量算路。

边界：契约保证的是*同一实例多线程查询* 与 *多实例并行*；**不**保证对 `NavDatabase` 做并发的 移动赋值/析构等生命周期操作（构造/移动/析构仍须单线程编排）。

## 2. 为什么这是「契约」而非「实现细节」

v2 的重写动机之一就是它满是 `static` 局部变量跨实例共享状态，多数据集场景下读到错误的顶点表。 v3 从第一天就立规矩：**无 `static`/全局可变状态，每个数据库实例自包含**。并发安全是这条规矩 的自然延伸——既然无共享全局态，同一只读实例被多线程读本就该安全。契约 B 把这一点显式化并 用测试锁死。

## 3. 唯一的共享可变状态：程序缓存

`FindRoutes` 需要按需加载机场的 CIFP 程序，这带来唯一的共享可变状态 `procedure_cache_` （`unordered_map<string, unique_ptr<CifpData>>`）。其余部分（图 `builder_`、`mora_`、`msa_`） 在 `Open` 后不可变，无需同步。

程序缓存有两种加载模式，分别用不同机制保证安全：

## 4. on-demand 模式：双检锁 + 指针稳定性

默认模式。`ProceduresFor`（`nav_database.cc`）用**双重检查锁**：

```
1. 加锁，查缓存；命中（含缓存的"无程序" nullptr）直接返回。
2. 未命中 → 【解锁】，在锁外解析 CIFP（磁盘 I/O 不占锁）。
3. 重新加锁，try_emplace 插入。
```

三个关键设计：

- **只锁 map 的查/插，不锁磁盘解析** → 并发查询**不同**机场可并行解析、互不阻塞。
- **返回指针跨 rehash 稳定**：缓存**只增不删**，值是 `unique_ptr<CifpData>`。map 扩容(rehash) 只搬移节点指针，堆上的 `CifpData` 对象地址不变 → `ProceduresFor` 返回的 `const CifpData*` 在实例生命周期内始终有效，可在锁外安全使用。
- **同 key 竞争用 `try_emplace`**：保留先到者，绝不覆盖 → 已返回的指针永不失效；晚到线程 解析的副本直接丢弃（无害的重复工作）。

mutex 放在 `unique_ptr` 里，是为了让 `NavDatabase` 保持可移动（`std::mutex` 不可移动，而 `Open()` 按值返回、需要移动操作）。

## 5. eager 模式：冻结后无锁读

面向 Web/批量并发算路——避免每次查询都 seek + 反序列化。`OpenCached(..., CifpLoad::kEager)` 在打开时 `FetchAll` 把全部机场程序反序列化进 `procedure_cache_`，然后**冻结**：

- 之后 `ProceduresFor` 只读已存在项，**跳过锁**（`cifp_eager_` 标志决定是否加锁）；
- **无插入 = 无 rehash = 无数据竞争** → 契约 B 天然成立，无需任何锁。

> ⚠️ **这不是「缺锁」，是「冻结后无锁读」**——审计时勿误判。两种模式复用同一个 `procedure_cache_` 容器，由 `cifp_eager_` 标志决定读路径是否加锁。

实测 eager 常驻 +102MB（192283 程序 / 764942 legs），on-demand 仅 +1.5MB.CLI 单次查询用 on-demand 最优；长驻服务用 eager。

## 6. CIFP 缓存的 Fetch：共享只读句柄 + 定位读

按需加载从统一 `.bfdb` 的 CIFP 段取段时，`CifpArchive::Fetch` 在 `Open` 时打开的**一个只读句柄** （指向整个 `.bfdb`）上做**定位读**（POSIX `pread` / Windows `ReadFile` + `OVERLAPPED`）：两者都按 显式绝对偏移读、**不改动任何共享文件位置**，故并发 Fetch 不同机场无数据竞争，**这一层不需要 任何锁**。段体引用直指 `CifpArchive` 持有的只读全局字符串池（一个 const `std::string`）， resolve 也无共享可变状态。

早期版本是「每次 Fetch 开一个独立 `ifstream`」——也 race-free，但每次调用都付一次 `open` 系统 调用。当年否决共享句柄的理由是「`ifstream`/`FILE*` 带一个共享文件游标 → seek+read 非原子 → 需要加锁」；`pread`/`OVERLAPPED` 的定位读把偏移作为参数传入、不碰游标，正好绕过这个游标问题， 于是共享句柄既省掉 per-fetch open、又不引入锁。句柄由 `PreadFile`（RAII）持有，使 `CifpArchive` 成为 move-only。

## 7. 搜索期自包含

A*/Yen 的所有状态都是**函数局部**的（`lib/core/graph/astar.cc`、`yen_kshortest.cc`）：g/geo/prev/ closed 数组、优先队列、Yen 的候选集与 deviation 索引，全部在栈上、每次调用独立。特别地：

- Yen 的 `SearchOptions` 里的 `node_blocked`/`edge_blocked` 禁集**按值捕获**，使 `std::function` 自包含、可跨线程持有，不指向任何循环局部集合；
- heuristic memoization 表也是每次搜索/每次 Yen 调用独立构造，不同并发查询各建各的。

所以搜索本身没有引入任何共享状态。

## 8. 验证：独立 tsan 预设 + 8 线程压测

契约 B 不是靠「看起来对」，而是用 **ThreadSanitizer**（独立的 `tsan` 预设，与 ASan 的 `debug` 分开）+ 8 线程并发集成测试锁死：

- 混同 key 压缓存竞争（多线程查同一机场）、异 key 压并行解析与 rehash（查不同机场）；
- eager 模式 8 线程并发算路，验证无锁读路径 race-free；
- CIFP `Fetch` 8 线程并发取不同机场。

**测试有效性也验证过**：早期去掉锁后 TSan 立即报 `procedure_cache_` 的 emplace/rehash 竞争， 证明测试真能抓竞争而非摆设。改任何并发相关代码后必须过 `ctest --preset tsan`。

## 9. 小结

- **一条契约**：Open 后只读 + const 并发查询；生命周期操作仍单线程。
- **唯一共享可变态**是程序缓存，两种模式各自安全：on-demand 双检锁（只锁 map、指针跨 rehash 稳定、try_emplace 保先到），eager 冻结后无锁读。
- **搜索期全函数局部**，禁集按值捕获，memo 表每次独立。
- **CIFP Fetch 在共享只读句柄上定位读**（pread / `ReadFile`+`OVERLAPPED`），不碰共享游标。
- 全部用 **tsan + 8 线程压测**锁死，且验证过测试能抓竞争。
