# BravoFinder

> [English](README.md) | **简体中文**

一个用现代 C++ 编写的航班航路查找工具。**这是第 3 版——一次完全重写，目前正在积极开发中。**

## 简介

BravoFinder 从导航数据（航路点、导航台、航路，以及 SID/STAR/进近程序）构建图，
在两个机场之间查找航路。与早期版本只计算纯地理最短路径不同，v3 的目标是做一个
**真实 / 合规的航路引擎**：产出的航路尊重真实世界的约束，例如航路方向性、
高低空航路分层、航段高度限制以及终端区程序。

## 状态

v3 正在从零重写，处于积极开发中。

**目前可用（里程碑 M1）**：航路网（enroute）寻路。工具加载 X-Plane 12 导航数据，
构建尊重航路方向性的有向图，用 A\* 在两个机场（或航路点）之间求最短路径。
例如 `KJFK KLAX` 会解析出一条约 2161 NM 的合理航路。

### 路线图

- **M1（已完成）** —— 航路网、A\* 搜索、`bf route` CLI。
- **M2** —— 可插拔约束：航路方向、高度band、高低空、MORA；Yen K-shortest 多候选航路。
- **M3** —— SID/STAR/进近程序（ARINC 424 / CIFP）、MSA。
- **M4** —— 程序 leg 精修，以及用于秒级启动的紧凑 `.bfdb` 缓存。

第一阶段不包含：Web API、地图可视化。

## 构建

需要 C++20 编译器与 CMake（3.21+）。依赖（Catch2、CLI11）通过 FetchContent 自动拉取。

```bash
cmake --preset debug      # 或 release
cmake --build --preset debug
ctest --preset debug      # 跑单元测试；集成测试需要数据（见下）
```

## 使用

```bash
# 查找航路（默认从 ./navdata 读取导航数据）
bf route KJFK KLAX
bf route EGLL LFPG --format json
bf route KSEA KBOS --data /path/to/xplane/data
```

端点为机场 ICAO 代码或航路点 ident，大小写不敏感。

## 导航数据

导航数据**不包含**在本项目中，需用户自行提供。数据受版权保护（Navigraph / Jeppesen），
仅限娱乐模拟用途，禁止再分发。请将本地数据放在 `navdata/` 目录下（已被 git 忽略）。

## 许可证

[MIT](LICENSE)。三方依赖见 [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md)。
贡献约定见 [docs/CONTRIBUTING.zh-CN.md](docs/CONTRIBUTING.zh-CN.md)。
