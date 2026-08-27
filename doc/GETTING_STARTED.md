# 运行第一个 Navigation2D 回归

## 前置条件

需要 C++20、CMake 3.16、Eigen3、yaml-cpp。Sophus 固定为 1.22.10：优先使用系统安装，
未安装时 CMake 通过浅克隆获取该版本。

## 1. 构建和单测

在 `third_party/navigation2d` 中执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## 2. 运行冻结回归

在 SweepNav 2D 根目录执行：

```bash
python3 tools/run_navigation2d_regression.py --tier smoke --jobs 2
python3 tools/run_navigation2d_regression.py --tier weekly --jobs 8
```

`smoke` 运行 2 个主场景，`nightly` 运行 4 个，`weekly` 运行全部 9 个。构建只进行一次，
case 使用独立进程并行执行。报告写入 `artifacts/navigation2d/results/`。

## 3. 运行单个 case

```bash
python3 tools/run_navigation2d_regression.py --case theta_corridor_00 --jobs 1
```

报告包含状态、终点位置/航向误差、碰撞数、重规划/急停/恢复次数、初始全局路径长度、
costmap digest 和轨迹 SHA256。

## 常见失败

- 地图分辨率不是 0.03 m：配置与输入地图不一致，构造阶段直接失败。
- YAML 缺字段或多字段：配置采用完整严格 schema，不接受隐式默认覆盖。
- 起终点处于致命/膨胀区域：全局规划返回无路径。
- 轨迹 hash 改变但阈值仍通过：先判断是预期算法变化还是非等价重构，不能直接更新基线。
