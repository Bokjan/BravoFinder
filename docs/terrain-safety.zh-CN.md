# 地形安全：MORA 网格与 MSA 扇区

> BravoFinder 的两种最低安全高度数据——航路级的 MORA 与终端级的 MSA——它们怎么建模、 怎么参与算路、又为什么用两套完全不同的数据结构。面向关心「这条航路飞得够不够高」的读者。 相关代码：`lib/core/domain/mora_grid.h`、`lib/core/domain/msa.h`、`lib/core/constraints/mora_constraint.h`、 `lib/io/nav_database_query.cc`。

## 1. 两种「最低安全高度」，两种尺度

真实运行里「最低安全高度」分两个尺度，BravoFinder 分别建模：

- **MORA(Grid Minimum Off-Route Altitude)**——**航路级、全球连续**。每 1°×1° 的格子一个值， 覆盖整个地球。回答的是「沿这条航路飞，过这一带地形至少要多高」。
- **MSA(Minimum Sector Altitude)**——**终端级、绕单点分扇区**。以某机场的一个中心 fix 为圆心， 按磁方位切成若干扇区，每扇区一个最低高度、一个半径。回答的是「在这个机场附近这个方向上、 这个距离内，至少要多高」。

尺度不同→**数据结构也不同**:MORA 是稠密全球网格（查任意坐标都有值），MSA 是稀疏的按机场 索引的扇区列表（只在有终端程序的机场附近有意义）。硬套一套结构会浪费——这是本文的主线。

## 2. MORA:1° 稠密网格，数组即结构

MORA 的访问模式是「给一个坐标，要它所在格子的值」，而且格子铺满全球。这天然就是一个**二维数组**， 不需要任何 map/树：

```
kLatCount = 180   // 纬度 -90..+89
kLonCount = 360   // 经度 -180..+179
cells_[(lat+90)*360 + (lon+180)]   // 行主序,int16 存 flight level
```

64,800 个 `int16` = 约 127 KB，常驻内存零压力。查询就是一次 `floor` 取整 + 一次数组下标：

```cpp
int16_t MoraAt(const Coordinate& c) const {
  const int lat = FloorToInt(c.latitude);   // 注意向负无穷取整
  const int lon = FloorToInt(c.longitude);
  const int idx = Index(lat, lon);
  return idx >= 0 ? cells_[idx] : 0;         // 0 = 该格无数据
}
```

两个容易踩的点：

- **向负无穷取整，不是向零截断**。南纬 12.3° 属于 `-13..-12` 这一格，`floor` 给 `-13`，而 `(int)(-12.3)` 会给 `-12`——错格。`FloorToInt` 专门处理负数分支。
- **`0` 表示「无数据」，不是「海平面」**。稀疏区域（大洋中央）没有 MORA，格子留 0。约路时把 0 当 「无下限」放行，而不是「最低 0 英尺」卡死——见 §4。

值单位是 **flight level（百英尺）**，和序列化直接对应：`.bfdb` 里就是 64,800 个 `int16` 连续块 （见 [binary-cache.zh-CN.md](binary-cache.zh-CN.md) 的 span 批量解码）。

## 3. MSA：按机场索引的稀疏扇区

MSA 的形状完全不同——它是「绕一个中心 fix、按磁方位切扇区」：

```cpp
struct MsaArc {
  int bearing_from;  // 扇区起始磁方位(顺时针到下一扇区起点为止)
  int alt_100ft;     // 该扇区最低安全高度(百英尺)
  int radius_nm;     // 扇区半径(海里)
};
struct MsaSector {
  Ident center;              // 扇区测量的中心 fix
  std::string airport_icao;  // 归属机场
  std::vector<MsaArc> arcs;
};
```

它只在有终端程序的机场附近存在，是**稀疏**的，所以不用网格——按机场 ICAO 线性存一组 `MsaSector`, 查询按 ICAO 过滤：

```cpp
std::vector<MsaSector> NavDatabase::MsaForAirport(const std::string& icao) const {
  std::vector<MsaSector> out;
  for (const MsaSector& s : msa_) {
    if (s.airport_icao == icao) out.push_back(s);
  }
  return out;
}
```

数据量小（每机场几个扇区、总量有限），线性扫足够，不值得再建索引。`MsaForAirport` 是 `const`、 无共享可变态，满足[契约 B](thread-safety.zh-CN.md)可并发查。

## 4. MORA 怎么参与算路：一个硬过滤约束

MORA 通过[可插拔约束框架](compliant-routing.zh-CN.md)接入搜索，是一条**硬过滤**(`MoraConstraint`)。 逻辑很克制：只有当查询给了巡航高度区间时才生效，且只在「连区间顶都够不着 MORA」时才 block:

```cpp
EdgeVerdict Evaluate(const EdgeContext& ctx, const RouteRequest& request) const override {
  if (!request.altitude.has_value()) return EdgeVerdict::Allow();  // 没给高度,不管
  const int16_t mora = grid_.MoraAt(ctx.to_coord);
  if (mora == 0) return EdgeVerdict::Allow();                      // 未知格,无下限
  if (request.altitude->max_fl < mora) return EdgeVerdict::Block(); // 连顶都低于 MORA
  return EdgeVerdict::Allow();
}
```

即：一条边进入的格子若 MORA 高于你区间的**上限**，这条边就飞不安全，搜索直接看不到它。这样 选出的航路能全程保持在地形/障碍之上——而不只是几何最短。

## 5. 一个诚实的近似：MORA 是 MSL，FL 是气压高度

必须说清的语义边界：**MORA 是海平面(MSL)高度，而 flight level 是气压高度**，两者严格来说不能 直接比。BravoFinder 目前直接比较，作为一个**安全下限的合理近似**——宁可偏保守。代码注释里明确 标了这一点，不假装它是精确的。这符合项目「语义诚实」的一贯态度：近似就说是近似，不粉饰。

## 6. 为什么不合并成一套结构

有人会问：都是「最低安全高度」，为何不统一？因为**访问模式决定数据结构**:

| | MORA | MSA |
|---|---|---|
| 覆盖 | 全球稠密 | 机场附近稀疏 |
| 查询键 | 任意坐标 | 机场 ICAO + 方位/距离 |
| 结构 | 2D 数组(O(1) 下标) | 按 ICAO 的扇区列表（线性） |
| 参与算路 | 硬过滤约束，逐边求值 | 查询期信息输出，不进搜索 |

MORA 每条边都要查(必须 O(1))，MSA 只在展示终端信息时查（线性够用）。合并只会让稠密的那半 背上 map 开销、稀疏的那半塞进浪费的网格。分开是「结构跟着访问模式走」的直接结果。
