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

## 能力边界与后续路线

本实现是适合二维差速底盘、静态或缓慢变化 costmap 的 **MPPI 基线控制器**，不是完整的
动态环境自主导航系统。它与传统 DWA 的共同点是 rollout 后评价候选轨迹；区别在于每一条
MPPI 样本均对整段 `(v_t, w_t)` 控制序列加入独立噪声，并以路径积分权重更新整段控制序列，
而非为一条候选轨迹固定一个 `(v, w)`。

当前实现的明确限制：

- 状态模型仅包含平面位姿，控制为 `(v, w)`；未建模底盘时延、轮胎/地面打滑、速度闭环误差
  或制动动力学。
- 障碍只来自当前时刻的 costmap；没有检测、跟踪和预测动态障碍，也没有不确定性传播。
- 速度/加速度主要通过逐步裁剪和软成本处理；未将碰撞距离、可行走廊或制动可达性作为优化器的
  硬约束。
- 碰撞检查基于离散 rollout 时刻和 costmap；末端的 `CollisionImminent` 是必要的独立安全
  刹车，但不构成连续时间安全证明。
- 固定的路径前视点、距离门槛和 critic 权重是工程启发式；单次迭代的小批量随机采样也不能稳定
  完成“绕障左/右”这类多拓扑决策。

因此，不能仅因控制器名为 MPPI 就将其视为动态人群或高速平台的最终方案。对于本项目的当前
2D 清扫/巡航目标，它是合理的快速基线；若要面向真机复杂场景，推荐按以下顺序演进：

```text
LocalMap（静态障碍） + obstacle tracks / future predictions
                         │
全局/覆盖路径 ─→ 局部可行走廊与多拓扑候选
                         │
              NMPC / MPCC（硬状态、输入与安全距离约束）
                         │
        独立 CBF 或 braking safety filter ─→ 底盘速度闭环
```

1. 先完善滚动局部地图、机器人轮廓的连续碰撞检查、控制时延和制动距离建模。
2. 对动态障碍输出带时间戳的轨迹及协方差；用 chance-constrained MPC 或 Safe-Horizon MPC
   处理预测不确定性，而不是把目标只画入一张瞬时 costmap。
3. 用局部可行走廊和多初值/多拓扑候选解决窄通道及左右绕行；在此基础上采用 NMPC 或 MPCC，
   将输入、输入变化率、曲率和安全间距表达为硬约束。
4. 在优化器后保留独立安全层。优化失败、定位/感知超时、或预测制动不可避免碰撞时，安全层必须
   输出零速度或受控制动，不能依赖 critic 权重“恰好足够大”。
5. 只有在未知地形或仅视觉感知成为主要瓶颈时，再引入学习式可通行性/局部参考模型；学习模型应
   优先辅助感知和参考生成，而不直接替代安全控制。

### 开源参考实现

- [Nav2 MPPI Controller](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/):
  面向通用 ROS 2 移动底盘的成熟 MPPI 基线；适合比较 critic 语义、批量 rollout 与恢复机制，
  但不应把动态 costmap 等同于动态障碍预测。
- [TUD-AMR mpc_planner](https://github.com/tud-amr/mpc_planner): 本项目向约束优化升级时的首选
  参考。它提供 MPCC、曲率感知 MPC、静态/动态障碍约束、概率/安全时域障碍规避和多拓扑候选，
  并支持使用开源 acados 生成在线 C++ 求解器。
- [ViPlanner](https://github.com/leggedrobotics/viplanner) 与
  [Wild Visual Navigation](https://github.com/leggedrobotics/wild_visual_navigation): 面向视觉语义和
  未知地形的学习式局部导航/可通行性方向；适合未来感知模块，而非当前二维 costmap 控制器的直接
  替换。
