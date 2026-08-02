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
- **Formatting**: enforced by clang-format (root `.clang-format`, `BasedOnStyle: Google`). Run `clang-format -i` on every changed `.h`/`.cc` before committing, and ensure `clang-format --dry-run --Werror` passes. A local `pre-commit` hook (`tools/hooks/pre-commit` → `tools/check_clang_format.py`) enforces this once `core.hooksPath` is set to `tools/hooks`. Every `if`/`else`/`for`/`while` body uses braces, including single-line bodies.
- **Language standard**: C++20.
- **No magic numbers**: reference named engine constants instead of hardcoding invariants.

### 2.1 Naming and Files

- **Source file names**: snake_case; headers `.h`, implementations `.cc` (e.g. `coordinate.h`, `nav_graph.cc`, `a_star.cc`).
- **Header guard**: use `#pragma once`.
- **Namespace**: `bf` throughout.
- Types `PascalCase`, variables `snake_case`, constants `kPascalCase`, member variables with trailing underscore `member_` (per Google convention).

### 2.2 Error Handling

- Use the in-house `bf::Result<T, E>` (`libs/engine/core/result.h`); do not use `std::expected` or `tl::expected`.
- Expected failures (e.g. "no route found") go through `Result`; exceptions are reserved for truly exceptional situations.
- No raw `new`/`delete` (use RAII / smart pointers), no `goto`, no catch-by-value.

---

## 3. Branching and Versioning

- **Branching**: single-developer workflow; commit directly to the `v3` branch, keeping a linear history; no feature branches. (The old version is preserved on the `v2` branch.)
- **Versioning**: the version number is `MAJOR.MINOR.PATCH` in shape, but the policy is **not** strict SemVer. While v3 is under active development (interfaces not yet frozen, no external consumers pinned to a stable API), MAJOR stays fixed at `3` absent a major shift in the project; a large new feature or a breaking fix bumps MINOR; any other code change, bugfix, or tweak bumps PATCH. Releases are tagged with git tags (e.g. `v3.15.1`).
- **CHANGELOG**: not maintained separately for now; relies on commit history; generated at formal release time.

---

## 4. License and Data Compliance

- This project is **dual-licensed**. Files under `libs/engine/` are distributed under the **GNU Lesser General Public License v3.0-or-later** (see `libs/engine/LICENSE`); all other files are **MIT** (see `LICENSE.MIT` at the root).
- **Inbound = outbound**: contributions to `libs/engine/` are licensed under LGPL-3.0-or-later; contributions elsewhere are licensed under MIT. Do not submit code whose license is incompatible with the target directory — in particular, **do not** introduce any GPL-only dependency into `libs/engine/`.
- All third-party dependencies use permissive licenses (see `THIRD_PARTY_LICENSES.md`) and are fetched at build time via FetchContent; **their source is not committed to this repository**.
- **Navigation data compliance**: Navigraph / Jeppesen data is copyrighted and may not be redistributed. Real `.dat` / `.bfdb` data is **never committed** (blocked by `.gitignore`). Local real data lives in `navdata/` (ignored).

## 5. AI-Assisted Development

This project is developed with AI assistance as the default, not the exception. The recommended division of labor:

- **Implementation**: use an AI coding agent (an LLM agent) to write the actual code. The project's `CLAUDE.md` records the hard conventions, invariants, and minefields an agent must follow when working in this repo.
- **Research and design**: investigation and design of an implementation approach should, in principle, use a frontier model — e.g. GPT-5.6-Sol, Claude Opus 5, or an equivalent flagship. The upfront reasoning quality gates the rest of the work, so align on the approach before implementing.
- **Pre-send review**: before opening a PR or filing an issue, have the change reviewed by **two different mid-tier models**. Disagreements between them surface blind spots a single reviewer misses; reconcile them before sending.

These are recommendations about model selection, not a substitute for the conventions above — an AI agent still follows the commit, code-style, and license rules in §1, §2, and §4.

---

## 6. Pre-PR Checklist

Run through this list before opening a PR. CI and the local hooks catch most of it, but checking locally first saves a review round-trip.

- [ ] **clang-format clean**: `clang-format -i` applied to every changed `.h`/`.cc`; `clang-format --dry-run --Werror` passes (or the `pre-commit` hook is active via `git config core.hooksPath tools/hooks`).
- [ ] **Braces on every body**: all `if`/`else`/`for`/`while` bodies use `{}`, including single-line ones.
- [ ] **No magic numbers**: engine invariants use their named constants, not literals.
- [ ] **Builds on all CI platforms**: a clean Linux build is not enough — the CI matrix spans macOS and Windows, which surface platform-specific warnings that Linux does not. Prefer waiting for CI to go green before requesting review.
- [ ] **Commit message conforms**: Conventional Commits type/scope/subject, English body, and no hard-wrapped paragraph (each paragraph is one logical line).
- [ ] **PR description matches the code**: every claim in the description — data filtering, test results, behavior — is backed by the diff; reconcile any internal inconsistencies.
- [ ] **Test results stated honestly**: if cases SKIP because the required data is not present, say so; do not report "all tests pass" when the run actually skipped cases.
