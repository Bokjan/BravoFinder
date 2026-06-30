# BravoFinder

> [English](README.md) | **简体中文**

一个用现代 C++ 编写的航班航路查找工具。**这是第 3 版——一次完全重写，目前正在积极开发中。**

## 简介

BravoFinder 从导航数据（航路点、导航台、航路，以及 SID/STAR/进近程序）构建图，
在两个机场之间查找航路。与早期版本只计算纯地理最短路径不同，v3 的目标是做一个
**真实 / 合规的航路引擎**：产出的航路尊重真实世界的约束，例如航路方向性、
高低空航路分层、航段高度限制以及终端区程序。

## 状态

v3 正在从零重写。里程碑见下；API、CLI 与数据格式支持仍在演进中。

### 第一阶段计划

- **`core/`** —— 领域库：领域模型、紧凑的 CSR 图、A* + Yen K-shortest、
  可插拔约束。无全局/静态状态；`bf::Result<T, E>` 错误处理。
- **`bf` CLI** —— `bf build`（解析导航数据并生成紧凑的 `.bfdb` 缓存）与
  `bf route`（快速查询候选航路）。
- **数据源** —— X-Plane 12 native `.dat`（含 ARINC 424 程序解析）。

第一阶段不包含：Web API、地图可视化。

## 构建

需要 C++20 编译器与 CMake。依赖（Catch2、CLI11）通过 FetchContent 自动拉取。

```bash
cmake --preset debug
cmake --build --preset debug
```

## 导航数据

导航数据**不包含**在本项目中，需用户自行提供。数据受版权保护（Navigraph / Jeppesen），
仅限娱乐模拟用途，禁止再分发。请将本地数据放在 `navdata/` 目录下（已被 git 忽略）。

## 许可证

[MIT](LICENSE)。三方依赖见 [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md)。
贡献约定见 [docs/CONTRIBUTING.zh-CN.md](docs/CONTRIBUTING.zh-CN.md)。
