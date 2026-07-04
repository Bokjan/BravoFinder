# 性能测试工具与复现方法

本目录固化 `docs/performance.zh-CN.md` 里性能数字的测量工具与步骤，方便以后换机器/换版本
一键复现。数字本身与解读在那篇文档，这里只讲**怎么跑**。

`route_bench.cc` 不入默认构建（`BRAVOFINDER_BUILD_BENCH` 默认 OFF），不污染 `bf` / 测试 / CI。

## 0. 前置

```bash
cmake --preset release && cmake --build --preset release
# 建一份统一缓存供所有基准复用（换 AIRAC 才需重建）
./build/release/apps/cli/bf build navdata -o /tmp/nav.bfdb
```

## 1. 启动时间：冷启动 vs 缓存加载（端到端墙钟）

进程级墙钟（含启动/退出），各跑 5 次取稳定值：

```bash
BF=./build/release/apps/cli/bf
for i in $(seq 5); do /usr/bin/time -p $BF route KJFK KLAX --data navdata >/dev/null; done  # 冷启动：解析+建图
for i in $(seq 5); do /usr/bin/time -p $BF route KJFK KLAX --db /tmp/nav.bfdb >/dev/null; done  # 缓存加载
```

## 2. 内存峰值 RSS

```bash
# Linux：Maximum resident set size 一行
/usr/bin/time -v $BF route KJFK KLAX --db /tmp/nav.bfdb 2>&1 | grep 'Maximum resident'
/usr/bin/time -v $BF route KJFK KLAX --db /tmp/nav.bfdb --cifp-load eager 2>&1 | grep 'Maximum resident'
# macOS 用 /usr/bin/time -l（字段名 "maximum resident set size"）
```

## 3. 纯搜索耗时（本工具）

剥离启动噪声，进程内 `OpenCached` 一次 + 循环 `FindRoutes`（10 城市对 × 30 轮，
`steady_clock` 只包 `FindRoutes`），报告 k=1/3/5/10 的 ms/search：

```bash
cmake --preset release -DBRAVOFINDER_BUILD_BENCH=ON && cmake --build --preset release --target bf_route_bench
./build/release/bench/bf_route_bench /tmp/nav.bfdb        # 默认 30 轮
./build/release/bench/bf_route_bench /tmp/nav.bfdb 100    # 可选：加轮数降噪
```

### 3a. 优化分解（baseline / +memoize / +Lawler）

三版对照只差算法层的 4 个文件；把它们 checkout 到对应 commit、重编 `bf_core`、重链本基准即可
（`FindRoutes` 接口跨三版一致）：

```bash
ALGO="core/graph/astar.cc core/graph/astar.h core/graph/yen_kshortest.cc core/graph/yen_kshortest.h"
for c in 2918c86:baseline ee3afb4:memoize f7a42c9:lawler; do
  commit=${c%%:*}; label=${c##*:}
  git checkout "$commit" -- $ALGO
  cmake --build --preset release --target bf_route_bench >/dev/null
  echo "===== $label ($commit) ====="
  ./build/release/bench/bf_route_bench /tmp/nav.bfdb
done
git checkout HEAD -- $ALGO   # 还原，务必执行
```

- `2918c86` — baseline（Yen，无 heuristic memoization、无 Lawler）
- `ee3afb4` — +memoize（多目标 heuristic 跨 spur memoization）
- `f7a42c9` — +Lawler（再叠加 Lawler；与当前 HEAD 算法等价）

> `git checkout <commit> -- <files>` 会把历史版本**同时**放进工作区和暂存区。跑完务必
> `git checkout HEAD -- $ALGO` 还原，否则会误提交旧算法文件。

## 4. profile（gprof，指认固有热点）

gprof 需要 `-pg` 全量编译，故不走 CMake 库，直接把 `core/`+`io/` 的源码与一个最小驱动一起
编译（`-I build/<preset>/core` 找生成的 `version.h`）：

```bash
cat > /tmp/prof_drv.cc <<'CPP'
#include "core/routing/route_request.h"
#include "io/nav_database.h"
int main(int argc, char** argv){
  auto nav = bf::NavDatabase::OpenCached(argc>1?argv[1]:"navdata/nav.bfdb");
  if(!nav) return 1;
  bf::RouteRequest r; r.departure="KJFK"; r.arrival="KLAX"; r.k=10;
  (void)nav.value().FindRoutes(r);                 // warmup
  for(int i=0;i<400;++i){ auto x=nav.value().FindRoutes(r); (void)x; }
  return 0;
}
CPP
g++ -std=c++20 -O2 -pg -I. -Ibuild/release/core /tmp/prof_drv.cc \
    $(find core io -name '*.cc') -o /tmp/prof_drv
cd /tmp && ./prof_drv /path/to/nav.bfdb && gprof /tmp/prof_drv gmon.out | head -20
```

gprof 把内联进 A* 主循环的边松弛/g 更新/堆操作都归到 `RunMultiSearch`，且不采样
malloc/系统调用，故其占比比按调用栈采样的工具（perf、macOS `sample`）更高；量级结论一致：
A* 遍历本身是绝对热点。
