# 约束 MPC / MPCC 控制器

`MpcController` 是生产控制器编排入口。主后端是生成的 acados MPCC；求解器不可用、失败、
超过 deadline 或独立安全复核失败时，确定性降级到从 Nav2 上游数值核心适配的 MPPI，再降级到
RPP，最后输出停车。旧 shooting/beam-search 已从运行链路和配置中删除。

每个周期在整段 `(v, w)` 序列上优化独轮车模型，并在每个阶段施加：速度、线/角加速度、静态
costmap footprint、预测动态障碍椭圆安全区等约束。代价包含 MPCC 风格轮廓误差、路径切线航向
误差、进度、控制与控制变化率、膨胀 costmap 成本；路径曲率会将期望速度限制为
`sqrt(max_lateral_acceleration / curvature)`。

## 动态障碍和概率安全

调用方通过 `NavigationSystem::UpdateDynamicObstacles()` 提供带速度的 `PredictedObstacle`。
预测中心为 `(x + vx*t, y + vy*t)`，安全椭圆半轴为机器人半径、障碍半径和安全边距之和，再加上
`dynamic_sigma_scale * sigma_x/y`。进入椭圆的候选轨迹不可行，而非只增加软成本。该模型是独立轴
协方差的保守近似，不是完整 Safe-Horizon MPC 的场景采样实现。

## acados 边界

`solver` 支持 `acados` 和 `mppi`。acados 后端复用上一周期最优状态/控制序列并向前平移 warm
start，报告求解时间、SQP 迭代和 KKT residual；超过 `deadline` 的迟到解不会下发。完整的静态
凸走廊和 Guidance Planner 多拓扑接入按本文后续章节实现，不能用三个角速度初值冒充。

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
独立复核；未编译 acados 后端时会安全回退到 MPPI。

benchmark JSON 同时输出 `controller_solve_samples` 和
`controller_solve_p50_us/p95_us/p99_us`。计时范围仅为 `LocalController::Compute()`，不包含
传感器、costmap、全局重规划、仿真和控制器外部的独立安全过滤器。
