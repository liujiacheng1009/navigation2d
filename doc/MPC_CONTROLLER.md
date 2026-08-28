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

## 静态安全走廊与 footprint

DecompUtil 固定在提交 `b0836c7228d19f0fa97282c584b55adf642279da`。每个控制周期只截取预测
时域覆盖的参考段和局部障碍点，生成凸多面体；半空间边界使用当前参考 yaw 下的多边形 footprint
支撑函数内缩。新版生成 solver 默认提供 8 个走廊半空间槽，这些是 acados 非线性规划的硬约束，
而不是求解后才检查 costmap。圆形 footprint 使用半径支撑函数。

## TUD Guidance 多拓扑

TUD Guidance Planner 固定在提交 `2c4188371e18e2fb3d083e0867b5e4d537a42860`。由于上游实现依赖
ROS/ros_tools，它作为运行时 producer，通过 `UpdateGuidanceCandidates()` 向 ROS-free 核心提交按
质量排序、带 topology ID 和 age 的路径。核心为候选维护独立 acados capsule，并行求解后仍按输入
顺序选择第一个安全可行解；过期候选被丢弃。这一契约避免复制一个缩水 PRM 冒充上游 Guidance。

## 独立安全层

`CollisionMonitor` 直接消费 LaserScan/PointCloud 命中点，不读取 costmap。它实现 stop、slowdown、
time-to-collision approach、源超时停车及触发/释放防抖。该过滤位于所有控制后端之后，因此 acados、
MPPI、DWA、RPP 和恢复动作不能绕开它。它是软件安全层，不宣称功能安全认证。

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

benchmark JSON 同时输出控制器编排耗时、完整控制周期和纯 solver 的 P50/P95/P99，并输出
deadline miss、后端命令数、降级率、最小 TTC、线/角 jerk P95 及 Collision Monitor 干预数。
统一矩阵由根仓库 `tools/run_local_controller_benchmark.py` 执行。
