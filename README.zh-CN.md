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

**目前可用（截至里程碑 M3）**：工具加载 X-Plane 12 导航数据，构建尊重航路方向性
与高低空分层的有向图，用 A\* + Yen K-shortest 在两个机场（或航路点）之间查找航路。
机场通过其真实 SID/STAR 程序（从 ARINC 424 / CIFP 解析）接入航路网，无程序数据时
回退到直飞连接。例如 `KJFK KLAX` 会解析出一条类似飞行计划的航路
`KJFK DEEZZ5 TOWIN ... PGS BASET5 KLAX`，约 2160 NM。

单个已加载的数据库可被多线程并发查询。

### 路线图

- **M1（已完成）** —— 航路网、A\* 搜索、`bf route` CLI。
- **M2（已完成）** —— 可插拔约束：高度band、MORA 安全下限、高低空偏好；
  Yen K-shortest 多候选航路。
- **M3（已完成）** —— SID/STAR/进近程序（ARINC 424 / CIFP，全 23 种 path
  terminator）、终端区 MSA、基于程序的机场接入、程序信息在航路与 CLI 输出中标注。
- **M4** —— 程序 leg 精修（航向/弧/高度型 leg）、更完整的多程序 K-shortest，
  以及用于秒级启动的紧凑 `.bfdb` 缓存。

第一阶段不包含：Web API、地图可视化。

## 构建

需要 C++20 编译器与 CMake（3.21+）。依赖（Catch2、CLI11、RapidJSON）通过
FetchContent 自动拉取。

```bash
cmake --preset debug              # 或 release
cmake --build --preset debug      # 并行构建（用 --preset，不要用路径形式）
ctest --preset debug              # 单元测试始终运行；集成测试需要数据（见下）
```

另有 `tsan` 预设（ThreadSanitizer）用于验证并发安全：

```bash
cmake --preset tsan && cmake --build --preset tsan && ctest --preset tsan
```

## 使用

```bash
# 查找航路（默认从 ./navdata 读取导航数据）
bf route KJFK KLAX
bf route EGLL LFPG --format json
bf route KSEA KBOS --data /path/to/xplane/data

# 按巡航高度约束（启用高度band 与 MORA 过滤）
bf route KJFK KLAX --alt 350

# 偏好高空(Jet)或低空(Victor)航路；要求多条候选
bf route KJFK KLAX --level high -k 3

# 限定用于 SID/STAR 选择的出发/到达跑道
bf route KJFK KLAX --rwy-dep RW31L
```

端点为机场 ICAO 代码或航路点 ident，大小写不敏感。当机场有程序数据时，
航路及其 leg 会标注所用的 SID 与 STAR（以及共享同一衔接 fix 的可互换程序）。

## 导航数据

导航数据**不包含**在本项目中，需用户自行提供。数据受版权保护（Navigraph / Jeppesen），
仅限娱乐模拟用途，禁止再分发。请将本地数据放在 `navdata/` 目录下（已被 git 忽略）。

## 许可证

[MIT](LICENSE)。三方依赖见 [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md)。
贡献约定见 [docs/CONTRIBUTING.zh-CN.md](docs/CONTRIBUTING.zh-CN.md)。
