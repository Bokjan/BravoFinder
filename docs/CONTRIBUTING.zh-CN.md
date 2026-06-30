# 贡献指南 / 项目约定

> [English](CONTRIBUTING.md) | **简体中文**

本文件规定 BravoFinder v3 的开发约定。所有提交都应遵守。

> 说明：本项目文档优先使用中文；但**代码注释与 commit message 必须使用英文**（见 §2）。

---

## 1. 提交规范（Conventional Commits）

提交信息采用 [Conventional Commits](https://www.conventionalcommits.org/) 格式：

```
<type>(<scope>): <subject>

<body>

<footer>
```

### 1.1 type（必填）

| type | 用途 |
|---|---|
| `feat` | 新功能 |
| `fix` | 修复 bug |
| `docs` | 文档变更 |
| `refactor` | 重构（不改变外部行为） |
| `test` | 新增或修改测试 |
| `build` | 构建系统、依赖（CMake、FetchContent 等） |
| `perf` | 性能优化 |
| `style` | 代码格式（clang-format 等，不改逻辑） |
| `chore` | 杂项（.gitignore、配置等） |

### 1.2 scope（可选）

标明改动所属模块，取项目分层名：
`core`、`graph`、`io`、`loader`、`constraints`、`routing`、`cli`、`cache` 等。

### 1.3 subject（必填）

- **英文**，祈使句现在时（用 `add` 而非 `added` / `adds`）。
- 首字母小写，**结尾不加句号**。
- 简明扼要，一行说清这次改了什么。

### 1.4 body（按需）

- **英文**。解释**为什么**这么改（动机、背景），而非复述改了什么。
- **本项目特殊约定：body 段落内不换行。** 每个段落写成一整行（不做 72 列硬折行），
  段落之间用一个空行分隔。
- 简单改动可省略 body。

### 1.5 footer

- 保留 Claude 协作署名：
  ```
  Co-Authored-By: Claude <noreply@anthropic.com>
  ```
- 关联 issue（如有）：`Closes #12`。

### 1.6 示例

```
feat(graph): add A* with great-circle heuristic

The previous Dijkstra implementation explored the whole graph without any goal direction, which was wasteful for point-to-point queries. A* with an admissible great-circle heuristic guides the search toward the destination and returns the same optimal path much faster.

Co-Authored-By: Claude <noreply@anthropic.com>
```

```
fix(loader): resolve duplicate fix idents by region
```

```
build: wire up Catch2 and CLI11 via FetchContent
```

---

## 2. 语言规则

| 内容类型 | 语言 |
|---|---|
| 文档（docs/、README、本文件等） | **优先中文** |
| 代码注释（`.h`/`.cc` 内所有注释，含文件头、类/函数文档、行内注释） | **必须英文** |
| git commit message（subject + body） | **英文** |
| 标识符（类型、函数、变量名） | 英文（C++ 惯例） |

---

## 3. 代码风格

- **基底**：[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)。
- **语义**：遵循 [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/)
  （RAII、`enum class`、`span`/视图优于裸指针等）。
- **格式化**：clang-format 强制（根目录 `.clang-format`，`BasedOnStyle: Google`）。
  提交前应已格式化。
- **语言标准**：C++20。

### 3.1 命名与文件

- **源文件名**：snake_case，头文件 `.h`、实现文件 `.cc`
  （例：`coordinate.h`、`nav_graph.cc`、`a_star.cc`）。
- **头文件保护**：使用 `#pragma once`。
- **命名空间**：统一 `bf`。
- 类型 `PascalCase`、变量 `snake_case`、常量 `kPascalCase`、成员变量尾下划线
  `member_`（遵循 Google 约定）。

### 3.2 错误处理

- 使用自写的 `bf::Result<T, E>`（`core/result.h`），不使用 `std::expected` /
  `tl::expected`。
- 预期内的失败（如"算不出航路"）走 `Result`；异常仅用于真正异常的情形。
- 禁止裸 `new`/`delete`（用 RAII / 智能指针）、禁止 `goto`、禁止按值 catch 异常。

---

## 4. 分支与版本

- **分支**：单人开发，直接在 `v3` 分支提交，保持线性历史，不开功能分支。
  （旧版本保留在 `v2` 分支。）
- **版本号**：遵循 [SemVer](https://semver.org/)（MAJOR.MINOR.PATCH）。
  当前开发版本为 `3.0.0-dev`；里程碑达成后以 git tag 标记（如 `v3.0.0`）。
- **CHANGELOG**：暂不单独维护，依赖 commit 历史；正式发布时再生成。

---

## 5. 许可证与数据合规

- 本项目以 **MIT** 协议开源（见根目录 `LICENSE`）。
- 第三方依赖均为宽松许可（见 `THIRD_PARTY_LICENSES.md`），通过 FetchContent
  构建时拉取，**不将三方源码提交进仓库**。
- **导航数据合规**：Navigraph / Jeppesen 数据受版权保护、禁止再分发。
  真实 `.dat` / `.bfdb` 数据**绝不入库**（已由 `.gitignore` 拦截）。
  本地真实数据放 `navdata/`（被忽略）。
