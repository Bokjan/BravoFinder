# Contributing Guide / Project Conventions

This document defines the development conventions for BravoFinder v3. All commits should follow them.

> Note on language: the project front matter (README, this file, THIRD_PARTY_LICENSES) is maintained in **English**; in-depth technical articles under `docs/` are written in **Chinese** (`*.zh-CN.md`). **Code comments and commit messages must be in English**.

---

## 1. Commit Convention (Conventional Commits)

Commit messages follow the [Conventional Commits](https://www.conventionalcommits.org/) format:

```
<type>(<scope>): <subject>

<body>

<footer>
```

### 1.1 type (required)

| type | Purpose |
|---|---|
| `feat` | New feature |
| `fix` | Bug fix |
| `docs` | Documentation changes |
| `refactor` | Refactoring (no external behavior change) |
| `test` | Add or modify tests |
| `build` | Build system, dependencies (CMake, FetchContent, etc.) |
| `perf` | Performance improvements |
| `style` | Code formatting (clang-format, etc.; no logic change) |
| `chore` | Miscellaneous (.gitignore, config, etc.) |

### 1.2 scope (optional)

Indicates the module affected, taken from the project layering: `core`, `graph`, `io`, `loader`, `constraints`, `routing`, `cli`, `cache`, etc.

### 1.3 subject (required)

- **English**, imperative present tense (use `add`, not `added` / `adds`).
- Lowercase first letter, **no trailing period**.
- Concise: one line stating what changed.

### 1.4 body (as needed)

- **English**. Explain **why** the change was made (motivation, context), not a restatement of what changed.
- **Project-specific rule: no line wrapping within a body paragraph.** Write each paragraph as a single line (no hard wrap at 72 columns); separate paragraphs with a blank line.
- The body may be omitted for trivial changes.

### 1.5 footer

- If AI assistance was used, add a co-author trailer reflecting the actual tool (e.g. `Co-Authored-By: Claude <noreply@anthropic.com>`). Do not fabricate one when no AI was involved.
- Reference issues if any: `Closes #12`.

### 1.6 Examples

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

## 2. Code Style

- **Base**: [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).
- **Semantics**: follow the [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/) (RAII, `enum class`, spans/views over raw pointers, etc.).
- **Formatting**: enforced by clang-format (root `.clang-format`, `BasedOnStyle: Google`). Code should be formatted before committing.
- **Language standard**: C++20.

### 2.1 Naming and Files

- **Source file names**: snake_case; headers `.h`, implementations `.cc` (e.g. `coordinate.h`, `nav_graph.cc`, `a_star.cc`).
- **Header guard**: use `#pragma once`.
- **Namespace**: `bf` throughout.
- Types `PascalCase`, variables `snake_case`, constants `kPascalCase`, member variables with trailing underscore `member_` (per Google convention).

### 2.2 Error Handling

- Use the in-house `bf::Result<T, E>` (`lib/core/result.h`); do not use `std::expected` or `tl::expected`.
- Expected failures (e.g. "no route found") go through `Result`; exceptions are reserved for truly exceptional situations.
- No raw `new`/`delete` (use RAII / smart pointers), no `goto`, no catch-by-value.

---

## 3. Branching and Versioning

- **Branching**: single-developer workflow; commit directly to the `v3` branch, keeping a linear history; no feature branches. (The old version is preserved on the `v2` branch.)
- **Versioning**: the version number is `MAJOR.MINOR.PATCH` in shape, but the policy is **not** strict SemVer. While v3 is under active development (interfaces not yet frozen, no external consumers pinned to a stable API), MAJOR stays fixed at `3` absent a major shift in the project; a large new feature or a breaking fix bumps MINOR; any other code change, bugfix, or tweak bumps PATCH. Releases are tagged with git tags (e.g. `v3.15.1`).
- **CHANGELOG**: not maintained separately for now; relies on commit history; generated at formal release time.

---

## 4. License and Data Compliance

- This project is open-sourced under the **MIT** license (see `LICENSE` at the root).
- All third-party dependencies use permissive licenses (see `THIRD_PARTY_LICENSES.md`) and are fetched at build time via FetchContent; **their source is not committed to this repository**.
- **Navigation data compliance**: Navigraph / Jeppesen data is copyrighted and may not be redistributed. Real `.dat` / `.bfdb` data is **never committed** (blocked by `.gitignore`). Local real data lives in `navdata/` (ignored).
