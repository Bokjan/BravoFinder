# BravoFinder — AI agent 工作指南

> **本文件只面向在本仓库工作的 AI agent**，沉淀跨会话必须遵守的硬约定，自包含、
> 无需读设计稿即可守规矩。面向**使用者**的介绍与用法在 `README.md`（勿把使用者
> 信息搬进本文件）；本文件只讲"改代码时怎么做才对"。
>
> **文档地图**（各司其职，别混）：
> - `README.md` — 面向使用者：项目是什么、如何构建、CLI 用法、数据合规。
> - `CLAUDE.md`（本文件）— 面向 AI agent：开发硬约定、不变量、雷区。
> - `.notes/` — 本地工作文档（gitignore，不入库）：`README.md` 是目录索引，`DESIGN.md` 是完整设计稿，`plans/`/`records/`/`research/` 是历史计划与记录。需要背景时从 README 入手。
> - `docs/` — 面向他人的公开文档（CONTRIBUTING、算法说明等，入库）。

一句话背景（细节看 README / DESIGN）：真实/合规航路引擎，解析 X-Plane 12 native 导航
数据（含 ARINC 424 程序）建图，用 A\* + Yen K-shortest 求尊重航空约束的候选航路。

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
- ② 统一容器 `format_version`（唯一一个，magic "BFDB"，校验不符走 `Result::Err(kFormatMismatch)`）。三段（graph/cifp/detail）合一文件、共用此版本号，不设 per-section 版本。
- ③ 程序 semver + source_loader + AIRAC cycle 写进容器 header 作 provenance（`build` 已删除，X-Plane 专有字段）。
- **改缓存磁盘布局 → bump 容器 `format_version`；发布行为变更 → bump 程序 semver。**
  仅读取侧/内部函数改动不动布局，不 bump。

## 测试
- **真实数据，不 mock**。真实 AIRAC 数据放本地 `navdata/`（gitignore），
  经 `BRAVOFINDER_NAVDATA` 定位（默认 `navdata/`）；**缺失即 `SKIP`，绝不 mock 顶替**。
- 单测用人工构造的最小真实格式样例（不是 mock 对象）。
- 无 CIFP 程序的机场测试用 **KIKR / KNWL**。
- 真实导航数据 / `*.dat` / `*.bfdb` **绝不入库**（Jeppesen 版权，禁止再分发）。

## 构建 / 测试（开发时）
> 面向使用者的完整 CLI 用法（`bf build` / `bf route` 各参数）见 README；此处只列
> 改代码后必跑的 preset。**只用 `--preset`，不用路径形式**。
```
cmake --preset debug      # Debug + ASan/UBSan + warnings-as-errors
cmake --preset release    # Release -O2
cmake --preset tsan       # ThreadSanitizer（并发验证）
cmake --build --preset <debug|release|tsan>
ctest --preset <debug|release|tsan>
```
依赖纯 CMake + FetchContent（Catch2 v3 / CLI11 / RapidJSON），不 vendor、不 vcpkg。

### 按改动范围选测试范围（省时间，见「测试」节的 scope 原则）
ctest 每个 case 独立进程；缓存 round-trip 测试（`bfdb:` / `cifp section:` 共 11 个）各 20–40s，
是墙钟大头，仅在**动缓存磁盘布局 / 序列化**时才需跑。日常按改动挑 preset：
```
ctest --preset unit    # 纯逻辑单测（~1.4s）：只改算法/约束/纯函数
ctest --preset quick   # 排除 cache 测试（~12s）：改路由/查询/约束但没碰缓存布局
ctest --preset debug   # 全量（~56s）：改了缓存布局/序列化，或发布前
ctest --preset tsan    # 改并发相关代码后必跑（契约 B）
```
（unit/quick 均基于 debug 配置，带 ASan/UBSan；只是用 `filter` 排掉慢的 cache 测试。）


## Git
- commit 用英文 **Conventional Commits**；body 段落内不换行；保留 Claude 署名。
- 复杂里程碑**先对齐再动手**；每轮大任务收尾做文档 / memory 沉淀。
