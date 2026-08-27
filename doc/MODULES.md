# 模块与源码索引

## 概览

```text
application → costmap → planning → control
      │           ▲                    │
      └── sensor ─┘                    └── Twist2d

simulation → application（仅 benchmark）
```

## application

- `navigation_system.h/.cc`：唯一产品编排入口；拥有 costmap、planner、controller、目标和调度状态。
- `navigation_config.h/.cc`：完整 YAML schema、类型转换和未知/缺失字段校验。
- `benchmark_runner.cc`：命令行适配器，仅链接 simulation，不进入产品库。

## costmap

- `grid_2d.*`：加载静态 occupancy grid，负责世界坐标与 cell 坐标转换。
- `layered_costmap.*`：静态层、激光障碍层、清障射线、指数膨胀、滚动窗口和碰撞查询。

## planning

- `global_planner.h`：`GlobalPlanner` 接口及 `Path` 契约。
- `dijkstra_planner.*`、`astar_planner.*`、`theta_star_planner.*`：独立全局规划策略。
- `grid_search.*`：三种策略共享的网格搜索基础设施，不解析配置字符串。
- `planner_factory.*`：唯一的 planner 名称到实现映射位置。

## control

- `local_controller.h`：`LocalController` 接口与 `Twist2d` 控制契约。
- `regulated_pure_pursuit.*`：路径跟踪、曲率/代价调速、加速度限制和前向碰撞预测。
- `dwa_controller.*`：低算力动态窗口速度采样、恒定控制 rollout 和多项代价评分。
- `mppi_controller.*`：控制序列采样、差速批量 rollout、critics 评分和重要性加权更新。

## geometry 与 sensor

- `geometry/pose_2d.h`：以 `Sophus::SE2d` 作为 `Pose2d`，点坐标使用 Eigen。
- `sensor/observations.h`：产品输入的 `LaserScan` 和传感器坐标系 `PointCloud2d`。

## simulation

`NavigationSimulator` 生成雷达观测、积分底盘真值并注入动态障碍。它可以依赖产品核心；
产品核心禁止反向依赖 simulation。
