# Navigation2D 架构与控制循环

## 产品数据流

```text
静态 occupancy grid ───────────────┐
LaserScan / PointCloud2d ──> layered costmap
                                   │
当前 Pose2d + goal ────────────────┼──> GlobalPlanner ──> Path
                                   │                         │
实测 Twist2d ──────────────────────┴────────> LocalController
                                                             │
                                           碰撞预测/进度恢复 ─┤
                                                             ▼
                                                          Twist2d
```

## 所有权

`NavigationSystem::Impl` 独占静态/动态 costmap、planner、controller、目标、当前路径和调度状态。
调用方拥有传感器、定位、时钟和底盘。所有输入通过值或 const 引用进入，核心没有后台线程。

## 每周期顺序

1. 新观测更新障碍层、射线清障并重新膨胀。
2. 观测变化、路径为空或到达周期时触发全局重规划。
3. 先检查 XY；进入位置容差后原地收敛 yaw。
4. 检查进度，超时后进入有界倒车/旋转恢复。
5. controller 计算候选速度并执行前向碰撞预测。
6. 返回状态和命令，由外部底盘执行。

## 全局规划

Dijkstra 使用零启发项；A* 使用欧氏启发项；Theta* 在 A* 扩展中对 parent 做视线松弛，
生成任意角路径。三者共享网格访问、代价和防切角规则，但分别通过 `GlobalPlanner` 实现，
选择逻辑只存在于 factory。

## 局部控制

RPP 从全局路径选择随速度变化的前视点，依据航向误差、曲率和障碍代价调速。MPPI 采样
整段 `(v,w)` 控制序列，用差速模型批量前推，经过多项 critics 评分后按路径积分权重更新
nominal sequence；每周期执行首个控制并将余下序列 warm-start 到下一周期。

## 仿真边界

`simulation/` 才负责生成扫描、移动障碍、真值碰撞和位姿积分。任何产品代码读取仿真真值，
或 benchmark 绕过 `NavigationSystem` 直接生成“成功轨迹”，都属于架构回归。
