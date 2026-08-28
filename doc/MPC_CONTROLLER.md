# 约束 MPC / MPCC 控制器

`MpcController` 是 Navigation2D 的实验性高阶局部控制器。它使用确定性 shooting/beam-search
后端，在保持纯 C++ 核心无 ROS、无专有求解器依赖的前提下，定义了可迁移到 acados 代码生成
后端的同一优化问题边界。

每个周期在整段 `(v, w)` 序列上优化独轮车模型，并在每个阶段施加：速度、线/角加速度、静态
costmap footprint、预测动态障碍椭圆安全区等约束。代价包含 MPCC 风格轮廓误差、路径切线航向
误差、进度、控制与控制变化率、膨胀 costmap 成本；路径曲率会将期望速度限制为
`sqrt(max_lateral_acceleration / curvature)`。

## 动态障碍和概率安全

调用方通过 `NavigationSystem::UpdateDynamicObstacles()` 提供带速度的 `PredictedObstacle`。
预测中心为 `(x + vx*t, y + vy*t)`，安全椭圆半轴为机器人半径、障碍半径和安全边距之和，再加上
`dynamic_sigma_scale * sigma_x/y`。进入椭圆的候选轨迹不可行，而非只增加软成本。该模型是独立轴
协方差的保守近似，不是完整 Safe-Horizon MPC 的场景采样实现。

## 多拓扑候选与 acados 边界

每轮以左、直行、右三个首控制初值同时展开，并在同一硬约束下保留各自可行序列，最后选择总代价
最小者。这是适用于单条全局路径的轻量多初值近似；它不替代 T-MPC++ 的全局 guidance planner。

运行时支持 `shooting` 和 `acados` 两个后端。acados 后端复用本文的状态、控制、stage cost、
静态/动态约束和 `PredictedObstacle` 输入；求解器不可用、RTI 失败或独立安全复核失败时自动回退
到 shooting 后端。

## acados 代码生成

仓库包含 `tools/generate_acados_mpc.py`，实际生成状态 `[x, y, yaw, v, w]`、输入
`[a, alpha]` 的 SQP-RTI 求解器；它的非线性约束为预测动态障碍安全椭圆，参数包括每阶段的
路径参考、速度参考和固定容量的障碍椭圆槽。默认生成 4 个槽，每阶段按与参考点的距离选择威胁
最大的障碍，未使用的槽会禁用；独立安全过滤器仍检查 tracker 输出的全部障碍。可通过
`--obstacle-slots` 调整生成容量。生成文件是构建产物，不提交到仓库：

```bash
export ACADOS_SOURCE_DIR=/path/to/acados
export PYTHONPATH="$ACADOS_SOURCE_DIR/interfaces/acados_template:$PYTHONPATH"
python3 tools/generate_acados_mpc.py --output generated/acados_navigation2d_mpcc
```

脚本会生成并编译 `libacados_ocp_solver_navigation2d_mpcc.so`。使用生成后端构建：

```bash
cmake -S . -B build-acados \
  -DNAVIGATION2D_ACADOS_GENERATED_DIR=$PWD/generated/acados_navigation2d_mpcc \
  -DNAVIGATION2D_ACADOS_ROOT=/path/to/acados/install
cmake --build build-acados --parallel
```

设置 `mpc.solver: acados` 即可启用。`dynamic_safety` 始终保留，负责求解失败和预测不一致时的
独立复核；未编译 acados 后端时会安全回退到 shooting。

benchmark JSON 同时输出 `controller_solve_samples` 和
`controller_solve_p50_us/p95_us/p99_us`。计时范围仅为 `LocalController::Compute()`，不包含
传感器、costmap、全局重规划、仿真和控制器外部的独立安全过滤器。
