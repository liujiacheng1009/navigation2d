# Navigation2D

面向差速扫地机器人的纯 C++ 2D 导航核心。该仓库保留 Nav2 方案中当前产品需要的
占用栅格、Dijkstra/A*/Theta* 全局规划和 RPP/DWA/MPPI/约束 MPC 局部控制，不依赖 ROS、DDS、
pluginlib、生命周期节点、action 或行为树。

## 结构

```text
application/  导航闭环与 benchmark 入口
config/       完整、严格校验的算法与安全参数
control/      RPP、DWA 与 MPPI 局部控制
costmap/      占用栅格、分层代价地图、滚动窗口与碰撞检测
geometry/     基于 Sophus/Eigen 的 SE(2) 几何契约
planning/     Dijkstra、A* 与 Theta* 全局规划
sensor/       激光扫描与二维点云输入契约
simulation/   仅供 benchmark 使用的动态障碍、雷达和底盘真值仿真
```

默认配置为 `config/navigation2d.yaml`。与 localization2d 一样，配置完全展开，不允许
include、环境变量或隐藏覆盖；缺少参数或出现未知参数会直接报错。导航静态地图与
localization2d 的子图统一使用 0.03 m 分辨率。

运行时使用静态层、激光障碍物层和指数膨胀层合成 master costmap，并提供 3 m × 3 m
滚动局部窗口。全局路径按代价重规划，RPP 控制器执行曲率/障碍代价调速、加速度约束和
前向碰撞预测；调度器负责周期重规划、预测停车、进度检测以及倒车/旋转恢复。

`selection.planner` 可选 `dijkstra`、`astar`、`theta_star`、`state_lattice`，默认使用
`state_lattice`；
`selection.controller` 可选 `rpp`、`dwa`、`mppi`、`mpc`（默认，优先 acados）。Dijkstra 适合作为稳定基线，A* 减少搜索，
Theta* 生成带安全净空的任意角路径；RPP 和 DWA 适合低算力平台，MPPI 对整段随机控制序列进行
差速模型 rollout，并用约束、路径、目标、航向、障碍和控制平滑成本优化局部轨迹。`mpc` 提供
MPCC 风格路径轮廓控制、曲率限速、静态 footprint 约束及带协方差膨胀的动态障碍预测约束。
`state_lattice` 在 `(x,y,yaw)` 上使用差速运动原语搜索，支持圆弧、原地旋转和可选倒车，并对
每条原语执行 swept-footprint 碰撞检查。

产品入口是 `application/navigation_system.h`：调用方通过 `SetGoal()` 设置目标，通过
`UpdateLaserScan()` 输入真实 2D 激光，或通过 `UpdatePointCloud()` 输入雷达坐标系下的
二维击中点，再把 localization2d/底盘提供的位姿和实测速度传给 `ComputeCommand()`。
核心只返回速度和导航状态，不生成传感器数据、不积分机器人位姿，也不访问仿真真值。

## 文档

- [文档首页](doc/README.md)：按使用、架构、API 和维护路径组织。
- [架构说明](doc/ARCHITECTURE.md)：产品控制循环、所有权和仿真边界。
- [配置参考](doc/CONFIGURATION_REFERENCE.md)：严格 YAML schema 与调参顺序。
- [开发与回归](doc/DEVELOPMENT_GUIDE.md)：9-case 多进程评测和轨迹 hash 门禁。

## 构建

依赖 C++20、CMake 3.16、Eigen3 和 yaml-cpp；Sophus 固定为 1.22.10，系统未安装时由
CMake 浅克隆获取：

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
