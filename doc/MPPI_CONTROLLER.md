# MPPI 局部控制器

## 来源与取舍

实现以 Navigation2 的 Apache-2.0 `nav2_mppi_controller` 为算法和参数语义基准，保留差速
motion model、控制序列 warm start、批量随机 rollout、动力学约束、critics 聚合、路径积分
重要性权重和输出轨迹碰撞复核。ROS 消息、TF、pluginlib、动态参数、可视化发布和多运动学
插件不进入纯 C++ 核心。

参考：

- <https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller>
- Williams 等人的 IT-MPC：<https://arxiv.org/abs/1707.02342>

## 每周期算法

1. 将上一周期 nominal control sequence 左移，作为 warm start。
2. 依据当前全局路径切线生成 informed proposal，与 nominal sequence 混合。
3. 对每个 batch 样本生成整段高斯 `(v,w)` 扰动。
4. 逐时间步施加速度/加速度约束，并用差速闭式模型 rollout。
5. 致命 costmap 状态直接标记为不可行。
6. 聚合约束、costmap、目标、目标航向、路径对齐/跟随/航向、前进偏好和控制平滑成本。
7. 以 `exp(-(cost-min_cost)/temperature)` 归一化样本权重，更新整段 nominal controls。
8. 平滑控制序列，复核首个命令的前向碰撞，执行首项并 warm-start 剩余序列。

## Critic 门控

机器人距目标较远时，以 PathAlign、PathFollow 和 PathAngle 为主，避免最终目标成本把局部轨迹
从可行全局路径上拉走。进入 1.4 m 后启用 Goal，进入 0.5 m 后启用 GoalAngle。这与 Nav2
MPPI 的 threshold-to-consider 思路一致。

## 确定性与性能

随机种子属于严格配置。相同地图、配置、路径和观测必须产生相同轨迹 hash。默认 scalar
CPU 配置为 256 条轨迹、30 个时间步和 1 次更新；当前 9-case 采用多进程并行回归。
提高 batch/time_steps 前必须测量控制周期预算，不能只依据离线成功率增加计算量。

## 与 RPP 的边界

RPP 保留为低算力、结构化静态环境的基线控制器。MPPI 是动态障碍和复杂局部决策的默认高级
控制方案。DWA 作为低算力、确定性传统控制器保留，但不与 MPPI 混为同一能力等级。
