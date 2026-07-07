# 领域建模与内存设计：值类型、Result、SmallVec

> BravoFinder 的领域类型为什么这么设计——不可变值类型、自研 `Result<T,E>`、无 `static`/无裸
> `new`、以及一个只在 build 期用一次的 `SmallVec`。这些不是零散的风格偏好，而是 v2→v3 重写的
> 核心动机的直接体现。面向想读源码、理解设计宪法的读者。相关代码：`lib/core/domain/`、
> `lib/core/result.h`、`lib/core/util/small_vec.h`。

## 1. 一条主线：v2 的 static 共享 bug

v3 是对 v2 的重写，而重写的头号动机之一是：**v2 满是 `static` 局部变量跨实例共享状态**。多数据集
场景下（比如同时装载两个 AIRAC cycle），一个数据库实例会读到另一个的顶点表——一个隐蔽且危险的
正确性 bug。

v3 因此立了几条**设计宪法**，本文讲的每个类型都是它的落点：

- **无 `static` / 全局可变状态**——每个 `NavDatabase` 实例完全自包含。
- **无裸 `new`/`delete`、无 `goto`、无按值 catch**——所有权靠 `unique_ptr`/值类型，不手工管理。
- **领域类型是不可变值类型**——不藏指针、不藏 lifetime，复制即安全。
- **预期失败走 `Result`，异常只留给真正异常的情形**。

这几条合起来，让「一个只读实例多线程并发查」([契约 B](thread-safety.zh-CN.md))成为规矩的自然
推论，而不是事后补的锁。

## 2. 值类型：自包含，不藏 lifetime

领域类型全是**拥有型值类型**，不持有指向别处的指针/引用。`Coordinate` 是极端例子——它连不变量
都没有，只是两个数：

```cpp
struct Coordinate {
  double latitude = 0.0;   // 正北为正
  double longitude = 0.0;  // 正东为正
  double DistanceTo(const Coordinate& other) const;  // haversine, 球面, 海里
};
```

`Ident` 稍复杂，但同样自包含——它是「航路点标识」：

```cpp
struct Ident {
  std::string ident;   // 如 "JFK"、"DEEZZ"
  std::string region;  // 两字母 ICAO 区域码,如 "K6"
};
```

**关键设计：ident 不是全局唯一的**。同一个代码在不同 ICAO 区域会复用。所以唯一标识一个点的是
`(ident, region)` **这一对**，它被建模成一个值类型，并作为全代码库统一的查找键(还特化了
`std::hash<Ident>` 用 boost 风格混合)。曾经用「仅 ident 取第一个」的查找，选中谁取决于加载顺序、
对用户不可见——v3 消除了它，宁可返回所有匹配点也不隐式挑一个。

值类型的红利：复制它们零 lifetime 风险，`NavDatabase` 复制/移动时不必担心悬垂指针，并发只读天然
安全——因为根本没有共享的可变间接层。

## 3. Result<T，E>：预期失败不用异常

「算不出航路」「数据缺失」这类**预期内**的失败不该抛异常——它们是正常控制流。项目用自研的
`bf::Result<T,E>`，一个 C++20 版的 `std::expected` 替身：

```cpp
template <class T, class E = Error>
class Result {
  static_assert(!std::is_same_v<T, E>, "value 与 error 类型必须不同");
 public:
  static Result Ok(T value);
  static Result Err(E error);
  bool has_value() const noexcept;
  const T& value() const&;
  const E& error() const&;
  T value_or(T fallback) const&;
};
```

几个刻意的设计：

- **API 名字照抄 `std::expected`**(`has_value`/`value`/`error`)——将来迁到 C++23 是机械替换。
- **不引第三方**。项目的依赖纪律是零 Abseil/Boost/LLVM;`std::expected` 要 C++23,所以自己写一个
  最小实现，底层是 `std::variant<T,E>`。
- **`Error` 带 `ErrorCode` 枚举**，调用方能按失败类别分支而不必解析字符串。缓存层最近就加了
  `kCacheCorrupt`/`kFormatMismatch` 来区分「文件损坏」和「版本不符」(见
  [binary-cache.zh-CN.md](binary-cache.zh-CN.md))。
- **`Result<void, E>` 偏特化**，给「要么成功无返回值、要么失败」的操作（如写文件）。

异常留给什么？真正异常的情形——比如内部不变量被破坏。日常的「这条查询没结果」一律走 `Result`。

## 4. SmallVec：为一处热点定制的内联小向量

`SmallVec<T, N>` 是个「前 N 个元素放栈上、超了才上堆」的小向量。它**只为一个场景而生**:
`GraphBuilder` 里的 `ident -> 该 ident 的所有顶点` 索引。

为什么值得专门造一个？数据说话——实测 AIRAC 2601 的分布：

> **98.88% 的 ident 只在 ≤ 4 个区域出现**。

于是 `N=4`：内联存储覆盖了几乎所有 ident，只有 1.12% 的长尾会溢出到堆一次(build 期，成本可
忽略)。用 `std::vector` 的话，这近 99% 的常见情形每个都要一次堆分配——纯浪费。

```cpp
// 该常量注释里写死了警告:
// N=4 来自实测分布,是 per-site 的,不是全局默认。
// 别的 map-of-vector 成员不要照搬,先测自己的分布。
inline constexpr int kIdentRegionInline = 4;
```

两个值得学的克制：

- **只实现用到的操作**（`push_back`、下标、`size`、range 迭代、move），不做成通用容器。最小面 =
  最少 bug。
- **限定 trivial 元素类型**(`static_assert(std::is_trivial_v<T>)`)。堆缓冲是裸 `malloc`，元素
  直接赋值进去、不 placement-new、不跑析构——非 trivial 类型会 UB.static_assert 把这个约束
  焊死；想放开就得加真正的构造/析构管线，而这个极简类型刻意不要。

`SmallVec` 是「局部优化要有数据支撑、且不外溢成通用抽象」的样本：它诞生自一次 profile，注释里
甚至警告别人别乱复用 `N`。

## 5. 内存分层：磁盘紧凑，运行时拥有

一个贯穿的模式：**磁盘布局和内存布局解耦**。字符串在 `.bfdb` 文件里是 `{offset, len}` 引用一个
字符串池（紧凑、去重），加载后**重建回拥有型 `Ident`**（全 SSO、无碎片、无 lifetime 风险）。文件层
为体积优化，运行时层为安全和访问速度优化，两者不互相迁就——细节见
[binary-cache.zh-CN.md](binary-cache.zh-CN.md)。

这正是本文主线的收尾：值类型让运行时安全简单，自研 `Result` 让失败显式，`SmallVec` 让热点省
分配，而它们都服务于同一个目标——一个自包含、无隐藏共享态、可并发只读的引擎。
