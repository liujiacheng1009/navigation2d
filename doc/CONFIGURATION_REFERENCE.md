# YAML 配置参考

## Schema 策略

默认配置为 `config/navigation2d.yaml`。所有 mapping 和字段必须完整出现；未知字段、缺失字段、
错误类型和不支持的算法名称均立即失败。配置不支持 include、环境变量或隐藏覆盖。

## `selection`

- `planner`：`dijkstra`、`astar`、`theta_star`、`state_lattice`。
- `controller`：`rpp`、`dwa`、`mppi`、`mpc`（默认，优先 acados）。

## `map` 与 `costmap`

- `resolution`：必须与输入地图一致，当前基线为 0.03 m。
- `robot_radius`：圆形 footprint 半径。
- `footprint`：机器人坐标系中的凸多边形顶点列表；空列表使用 `robot_radius`。State Lattice 会按
  yaw bin 预计算占据格并对整条运动原语做 swept-volume 检查。多边形配置下 `robot_radius`
  仍应设置为外接圆半径，供膨胀、动态障碍和保守诊断使用。
- `inflation_radius`、`inflation_cost_scaling`：膨胀范围和指数衰减。
- `obstacle_max_range`、`raytrace_max_range`：标障和清障射线范围。`raytrace_max_range` 必须不小于
  `obstacle_max_range`；激光的正无穷无回波会清除至该范围，但不会标记障碍。
- `local_window_width`、`local_window_height`：局部滚动窗口尺寸。

## `state_lattice`

`yaw_bins` 是 SE(2) 朝向离散数；`primitive_length` 是长直线/圆弧原语的目标长度，生成器还会
加入单栅格直线以保持完整连通性。`allow_reverse` 控制倒车原语，`reverse_penalty`、
`rotation_cost` 和 `cost_penalty` 分别惩罚倒车、原地旋转和沿原语积分的 costmap 势场。
`initial_heuristic_weight` 是 ARA* 初始次优界，`heuristic_weight_decrement` 控制每轮改善幅度；
搜索在各轮间复用 OPEN、INCONS 和 g 值。`max_expansions` 与 `max_planning_time` 是所有改善轮次
共享的硬资源预算；超限时返回已有安全解，否则明确失败，不返回半构造路径。反向 cost-aware
障碍启发式按 costmap digest、目标格、clearance 和代价权重缓存。
若 D* Lite 与 ARA* 都在资源预算内找不到 SE(2) 解，规划器自动调用 cost-aware 2D A*；
`global_plan_fallback_used` 明确报告该次降级。

## `path_smoother`

平滑器在 State Lattice 路径生成后优化数据保持、二阶平滑和障碍距离项。`max_deviation` 定义相对
原路径的硬安全走廊，`max_curvature` 是局部三点曲率上限，`min_clearance` 是障碍排斥开始距离；
每次更新还会按半栅格间距检查相邻线段的圆形 footprint。起终点、原地旋转及前后向 cusp 固定。
`max_step`、`max_iterations` 和 `tolerance` 控制确定性迭代收敛；`enabled` 可用于基准消融。

## `controller`

RPP 与公共运动限制使用期望线速度、lookahead 距离/时间、到点减速、最小转弯半径、最大
线/角速度、加速度和控制周期。`rotate_to_heading_min_angle` 决定先原地对齐还是直接跟踪。

## `dwa`

`horizon` 是恒定 `(v,w)` 前推时间，线/角 samples 决定动态窗口离散度，四个 weight 控制
全局路径、最终目标、障碍和速度偏好。DWA 面向算力受限平台，不替代 MPPI 的控制序列优化。

## `mppi`

`time_steps × control_period` 是预测时域，`batch_size` 是每轮控制序列样本数，
`iterations` 是重要性加权更新轮数。`temperature` 控制优质样本权重集中度，`gamma` 是
控制扰动成本；`vx_std`、`wz_std` 控制探索范围。其余 weight 对应约束、costmap、目标、
目标航向、路径对齐/跟随/航向、前进偏好和控制平滑 critics。

## `mpc`

`solver` 可选 `mppi` 或 `acados`；后者要求构建时提供生成 solver 和 acados 安装目录，
不可用、失败、超时或安全复核失败时回退到 MPPI，再回退到 RPP/停车。
`time_steps × control_period` 是约束时域，`deadline` 是 acados 迟到解拒绝阈值；
`dynamic_prediction_timeout` 是 tracker 预测的最大允许年龄。
`guidance_max_candidates` 限制并行 acados capsule 数，`guidance_candidate_timeout` 拒绝过期的
TUD Guidance 输出。
`contour_weight`、`heading_weight`、`progress_weight` 是 MPCC 路径轮廓、切线航向和进度项；
其余权重约束速度参考、控制、控制变化率和 costmap 成本。`max_lateral_acceleration` 决定曲率
限速；`dynamic_safety_margin` 与 `dynamic_sigma_scale` 分别为动态预测的几何余量和协方差倍率。

## `safety`、`scheduler` 与 `recovery`

- `collision_horizon`：执行命令前的前向碰撞预测窗口。
- `collision_monitor_*`：独立原始传感器安全层的 stop/slowdown 距离、降速倍率、approach 时域、
  数据源超时和触发/释放防抖周期。数据源超时直接停车。
- `goal_xy_tolerance`、`goal_yaw_tolerance`：完整 SE(2) 到达阈值。
- `global_replan_period`：周期全局重规划间隔。
- `progress_radius`、`progress_timeout`：卡住检测。
- `duration`、倒车/旋转速度：恢复行为参数。
- `dynamic_obstacle_radius`：benchmark 动态障碍模型半径，不应解释为真实感知参数。

## 调参顺序

先固定地图分辨率、footprint 与膨胀；再验证全局路径可行性；然后调控制器和运动限制；最后
调整重规划、进度检测与恢复。安全参数变化必须运行动态障碍和永久封路 case。
