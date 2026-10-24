# 模块与源码索引

## 概览

```text
ROS / exploration application → navigation orchestration → planning / control
                                                      └→ costmap
configuration YAML assembly → configuration values / exploration config
planning / control / costmap → configuration values / geometry values
simulation → application（仅 benchmark / replay）
```

当前仍保留 navigation2d_core 聚合 target；已新增 navigation2d_values 与
navigation2d_geometry 边界，尚未把所有概念层拆成独立 target。

## application

- `navigation_system.h/.cc`：唯一产品编排入口；拥有 costmap、planner、controller、目标和调度状态。
- `application/navigation_config.h`：旧 include 兼容入口；值定义在 `configuration/navigation_config.h`。
- `navigation_config.cc`：现有 YAML schema、类型转换和校验，分组解析仍待后续迁移。
- `route_preparation.*`：规划回退、角点验收、签名冷却和平滑流水线，不拥有执行状态。
- `command_stall_watch.*`：累计位姿/路线/测量速度证据、时间窗与周期预算的唯一写入者。
- `benchmark_runner.cc`：命令行适配器，仅链接 simulation，不进入产品库。

## exploration application 与 ROS 适配

- `application/exploration/entry_target.*`：按门宽、已知自由深度和尝试次数选择探入窗口，不消耗预算、不提交目标。
- `entry_coordinator.*`：入口锁、证据、尝试/爬行预算和入室判断；爬行返回阻塞、行驶或最后一个命令，适配器保持先发布末命令再处理退出。
- `escape_execution.h`：脱困命令、截止时间和返航意图的单一所有者；实时雷达修正仍复用 recovery_policy。
- `pose_motion_watch.*`：独立累计位姿运动与停滞时间窗，保留慢速入室保护，不以编码器转动代替平移进展。
- `dispatch_policy.*`：初始路线净空验收、目标超时与三类同步递归重试作用域；快照、目标提交和调度仍由节点编排。
- `return_coordinator.*`：返航目标/轨迹点几何及直返→轨迹→有界脱困→最终重试顺序；通过 ReturnActions 窄端口执行，不依赖 ROS 或共享任务 Context。
- `ros/exploration/pose_feedback.*`：定位/真值反馈转换、只读当前位姿、严格超过 0.05 m 的轨迹采样与累计里程；两个订阅路径共用单一状态所有者。
- `ros/exploration/debug_paths.*`：全局/局部/tour 消息构造和显示光标状态，不发布消息、不提交目标。
- `ros/exploration/map_observation.*`：世界坐标线段对实时占用图的查询，同时供显示和爬行判断使用；不归调试模块拥有。
- `ros/autonomous_explorer_node.cc`：仍承担任务/tour 编排、ROS 输入和动作执行，尚未完成薄适配验收。

## costmap

- `grid_2d.*`：加载静态 occupancy grid，负责世界坐标与 cell 坐标转换。
- `layered_costmap.*`：静态层、激光障碍层、清障射线、指数膨胀、滚动窗口和碰撞查询。

## planning

- `global_planner.h`：`GlobalPlanner` 接口及 `Path` 契约。
- `dijkstra_planner.*`、`astar_planner.*`、`theta_star_planner.*`：独立全局规划策略。
- `grid_search.*`：三种策略共享的网格搜索基础设施，不解析配置字符串。
- `planner_factory.*`：旧工厂兼容入口，委托 `application/algorithm_registry.*`。
- `path_geometry.*`、`route_identity.*`、`path_metrics.*`：导航路线的几何、身份与指标。
  `DensifyRoute` 与 grid_search 的 `DensifyPath` 保持各自原有零长度段/朝向语义。

## control

- `local_controller.h`：`LocalController` 接口与 `Twist2d` 控制契约。
- `regulated_pure_pursuit.*`：路径跟踪、曲率/代价调速、加速度限制和前向碰撞预测。
- `dwa_controller.*`：低算力动态窗口速度采样、恒定控制 rollout 和多项代价评分。
- `mppi_controller.*`：控制序列采样、差速批量 rollout、critics 评分和重要性加权更新。

## geometry 与 sensor

- `geometry/pose_2d.h`：以 `Sophus::SE2d` 作为 `Pose2d`，点坐标使用 Eigen。
- `geometry/path_2d.h`、`twist_2d.h`：路径和速度值，不依赖规划/控制接口。
- `geometry/robot_geometry.*`：进程机器人几何的安装、校验与只读查询，不负责 YAML。
- `configuration/vehicle_config.*`：YAML 到机器人/探索配置的适配；旧 geometry include 保留。
- `exploration/exploration_config.h`：独立探索配置值，不拉入 FrontierExplorer。
- `sensor/observations.h`：产品输入的 `LaserScan` 和传感器坐标系 `PointCloud2d`。

## simulation

`NavigationSimulator` 生成雷达观测、积分底盘真值并注入动态障碍。它可以依赖产品核心；
产品核心禁止反向依赖 simulation。

## 验证入口

`test/replay/README.md` 说明如何从真实 ROS 可执行目标构建冻结/候选受控回放，
并通过 A/A→A/B 检查轨迹、非计时输出及性能。该回放替换发布器，不代表 DDS 或板端验收。
