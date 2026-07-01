# BravoFinder — 项目协作规范

> 面向在本仓库工作的人与 AI 助手。完整设计在 `.notes/DESIGN.md`（本地稿，不入库），
> 本文件只沉淀**跨会话必须遵守的硬约定**，自包含、无需读设计稿即可守规矩。

BravoFinder 是一个真实/合规航路引擎：解析 X-Plane 12 native 导航数据（含 ARINC 424
程序），建图，用 A\* + Yen K-shortest 求尊重航空约束（航路方向、高低空、高度限制、
SID/STAR）的候选航路。CLI 子命令 `bf build`（预编译缓存）/ `bf route`（查询）。

## 语言
- 交流、注释、文档、commit 说明默认**简体中文**；文档优先中文。
- 例外保持原文：代码标识符、既有代码风格、技术专有名词/命令/API、**代码注释用英文**。

## 代码规范
- 现代 C++20；**Google C++ Style**（格式基底，clang-format `BasedOnStyle: Google`）
  + **C++ Core Guidelines**（语义正确性）。
- 文件名 `snake_case`，扩展名 **`.h` / `.cc`**；命名空间 `bf`。
- 错误处理用自写 `bf::Result<T, E>`（`core/result.h`）；预期失败（算不出航路、数据缺失）
  走 `Result`，异常只留给真正异常的情形。**无裸 `new/delete`、无 `goto`、无按值 catch、
  无 `static`/全局可变状态**（v2 的 static 共享 bug 是重写动机之一）。
- JSON 输出用 **RapidJSON `Writer`**（SAX 流式、自动转义），不手拼、不引 nlohmann。

## 线程安全契约 B（改并发相关代码务必遵守）
- `NavDatabase::Open()` 成功后实例**只读**，唯一例外是内部同步的程序缓存；
  `FindRoutes()` / `MsaForAirport()` 是 `const`，可多线程并发于同一实例。
- on-demand：`procedure_cache_` 由 `cache_mutex_` 双检锁守护，只锁 map 查/插、不锁磁盘解析；
  append-only + `unique_ptr` 值 → 返回指针跨 rehash 稳定。
- eager：`FetchAll` 在 Open 时填满后**冻结**，`ProceduresFor` 走无锁读（无插入=无 rehash=无竞争）。
- **eager 模式是"冻结后无锁读"，不是缺锁——审计时勿误判。**
- 改并发相关代码后**必须过 tsan 预设**：`ctest --preset tsan`。

## 版本纪律（三层）
- ① 程序 semver（CMake `project VERSION` → `core/version.h` 的 `kBravoFinderVersion` → `bf --version`）。
- ② 每类缓存 `format_version`（图 "BFDB"、CIFP "BFCP"，校验不符走 `Result::Err`）。
- ③ 程序 semver + source_loader 写进缓存 header 作 provenance。
- **改缓存磁盘布局 → bump 对应 `format_version`；发布行为变更 → bump 程序 semver。**
  仅读取侧/内部函数改动不动布局，不 bump。

## 测试
- **真实数据，不 mock**。真实 AIRAC 数据放本地 `navdata/`（gitignore），
  经 `BRAVOFINDER_NAVDATA` 定位（默认 `navdata/`）；**缺失即 `SKIP`，绝不 mock 顶替**。
- 单测用人工构造的最小真实格式样例（不是 mock 对象）。
- 无 CIFP 程序的机场测试用 **KIKR / KNWL**。
- 真实导航数据 / `*.dat` / `*.bfdb` **绝不入库**（Jeppesen 版权，禁止再分发）。

## 构建 / 测试
```
cmake --preset debug      # Debug + ASan/UBSan + warnings-as-errors
cmake --preset release    # Release -O2
cmake --preset tsan       # ThreadSanitizer（并发验证）
cmake --build --preset <debug|release|tsan>
ctest --preset <debug|release|tsan>
```
依赖纯 CMake + FetchContent（Catch2 v3 / CLI11 / RapidJSON），不 vendor、不 vcpkg。

## Git
- commit 用英文 **Conventional Commits**；body 段落内不换行；保留 Claude 署名。
- 复杂里程碑**先对齐再动手**；每轮大任务收尾做文档 / memory 沉淀。
