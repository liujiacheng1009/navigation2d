# Navigation2D

面向差速扫地机器人的纯 C++ 2D 导航核心。该仓库保留 Nav2 方案中当前产品需要的
占用栅格、NavFn 风格全局规划和 Regulated Pure Pursuit 跟踪，不依赖 ROS、DDS、
pluginlib、生命周期节点、action 或行为树。

## 结构

```text
application/  导航闭环与 benchmark 入口
config/       完整、严格校验的算法与安全参数
control/      Regulated Pure Pursuit 路径跟踪
costmap/      占用栅格、分层代价地图、滚动窗口与碰撞检测
planning/     NavFn 风格栅格最短路径规划
simulation/   仅供 benchmark 使用的动态障碍、雷达和底盘真值仿真
types.h      导航模块共用的最小数据类型
```

默认配置为 `config/navigation2d.yaml`。与 localization2d 一样，配置完全展开，不允许
include、环境变量或隐藏覆盖；缺少参数或出现未知参数会直接报错。导航静态地图与
localization2d 的子图统一使用 0.03 m 分辨率。

运行时使用静态层、激光障碍物层和指数膨胀层合成 master costmap，并提供 3 m × 3 m
滚动局部窗口。全局路径按代价重规划，RPP 控制器执行曲率/障碍代价调速、加速度约束和
前向碰撞预测；调度器负责周期重规划、预测停车、进度检测以及倒车/旋转恢复。

`selection.planner` 可选 `dijkstra`、`astar`、`theta_star`；
`selection.controller` 可选 `rpp`、`dwa`。Dijkstra 适合作为稳定基线，A* 减少搜索，
Theta* 生成带安全净空的任意角路径；RPP 适合结构化静态路径，DWA 通过动态窗口采样
并按路径、目标、障碍和速度评分局部轨迹。

产品入口是 `application/navigation_system.h`：调用方通过 `SetGoal()` 设置目标，通过
`UpdateLaserScan()` 输入真实 2D 激光，或通过 `UpdatePointCloud()` 输入雷达坐标系下的
二维击中点，再把 localization2d/底盘提供的位姿和实测速度传给 `ComputeCommand()`。
核心只返回速度和导航状态，不生成传感器数据、不积分机器人位姿，也不访问仿真真值。

## 构建

依赖 C++20、CMake 3.16 和 yaml-cpp：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

仓库根目录的 `tools/run_navigation2d_regression.py` 会只构建一次，然后用独立子进程
并行运行所有冻结 case。每个 case 直接加载数据集中的 occupancy grid，不启动容器，
也不存在 ROS domain、节点就绪或 action 超时竞争。

## 来源与许可

算法选择继承自 Open Navigation LLC 的 Navigation2 项目：NavFn 与 Regulated Pure
Pursuit。ROS 框架层已移除，代码按当前 2D 产品边界重新组织。原项目与本仓库许可见
`LICENSE`。
