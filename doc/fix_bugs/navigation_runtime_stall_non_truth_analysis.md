# 导航运行时停滞复现与非真值修复方案

## 1. 复现结论

在回退到 2026-09-03 提交后，使用 3 个并行 Compose worker、4 个合法起点
（seed `0/1/4/10`）完成 12 个 case 的基线回归。验收条件为离线 truth 栅格覆盖率
`>=0.90`、完成目标数 `>0`、返航误差 `<=0.10 m`，结果为 **9/12 通过**。

失败样本：

| Case | 覆盖率 | 返航误差 | 现象 |
|---|---:|---:|---|
| warehouse-0 | 1.0000 | 17.443 m | 返航首段纯旋转，弧长 0 |
| maze-0 | 0.6912 | 7.965 m | 前沿首段纯旋转，探索超时 |
| maze-10 | 1.0000 | 16.936 m | 返航首段纯旋转，弧长 0 |

三个失败日志都包含：

```text
requested=(0.000, +/-0.108)
published=(0.000, +/-0.108)
monitor_action=0
velocity=(0.000,0.000)
arc=0.00/<path_length>
```

这说明命令已发布且没有被 CollisionMonitor 拦截，但底盘没有产生可观测刚体运动。
失败点位于货架/窄道附近，首要假设是轮子或脚轮接触约束、轮子空转或低速转向无法
克服约束；现有日志没有轮速、IMU 和接触信息，不能把其中某一个未经测量的假设
当作唯一根因。

### 1.1 追加传感器复现实验：确认轮速空转

随后对 `warehouse-0` 单独启动容器并读取 `/cmd_vel`、`/wheel_joint_odom`、`/imu`、
`/scan` 和 `/ground_truth`。在停滞窗口直接观测到：

```text
/wheel_joint_odom: linear.x =  0.2386 m/s, angular.z = -0.0116 rad/s
/imu:              angular_velocity.z ≈ 0
/ground_truth:     linear.x = 0, angular.z = 0
```

此时导航日志仍显示首段 `arc=0`，CollisionMonitor 为 `0`。因此可以确认：轮速里程计
在轮子持续受驱动、但底盘被实体接触约束时报告了假运动；IMU 和真值均证明底盘刚体
没有旋转。根因不是角速度真值错误，也不是单纯的消息延迟，而是**轮速里程计不能
作为接触场景下的唯一运动反馈**。导航应使用 IMU/激光定位确认刚体运动，并把“轮速
非零 + IMU/定位不变”分类为打滑/接触停滞。

### 1.2 导致卡住的导航代码缺陷

控制器实现中存在确定性缺陷：`RegulatedPurePursuit::CollisionImminent()` 对所有
`linear == 0` 的命令直接返回 `false`。路径首段要求原地旋转时，它不检查当前
footprint 是否已经贴住或穿入障碍，也不触发脱困动作，于是会持续发布
`v=0,ω=±0.108`。一旦前一段曲线跟踪把底盘带到货架边缘，轮子会继续转而车体无法
转动，正好形成“轮速非零、IMU/真值不变”。“圆形 footprint 原地旋转不扩大占用集”
这个假设忽略了当前位姿可能已经处于接触状态。

停滞监视还有第二个缺陷：它把控制器命令当作有效进展，没有把 IMU/激光定位的连续
零增量升级为 `ROTATION_STALL`，因此恢复阶段只重放同一条首段路径。

## 2. 真值的正确边界

仿真中的 `/ground_truth` 是 Gazebo 真值位姿和 twist，可用于测试结束后的覆盖率、
返航误差和故障复盘。导航控制不能直接读取它。当前实现中探索节点直接订阅
`/ground_truth`，因此它属于“测试 oracle”，不满足真实导航链路约束；本次 `velocity=0`
只能证明仿真刚体没有运动，不能作为线上算法输入。

正确的数据流应为：

```text
轮编码器/轮速里程计 ─┐
IMU yaw rate ────────┼─> 定位/状态估计 ─> Navigation2D/RPP
激光 scan matching ─┘

/ground_truth ─> 仅离线 validator，不进入导航节点
```

## 3. 不使用真值的故障判别

导航节点保存命令、轮速里程计、IMU 和 Localization2D 位姿的短窗口：

| 轮速 | IMU yaw | 定位 yaw | 诊断 |
|---|---|---|---|
| 近零 | 近零 | 不变 | 命令链路/驱动执行失败 |
| 非零 | 近零 | 不变 | 轮子空转、接触约束或静摩擦 |
| 非零 | 非零 | 不变 | 定位更新异常 |
| 非零 | 非零 | 同向变化 | 正常旋转对齐 |

纯旋转命令只有在 IMU 或激光定位提供同向刚体旋转证据时才算有进展；不能只看轮速，
因为接触时轮子可能空转。该判断完全不需要 `/ground_truth`。

## 3.1 为什么规划器仍会把机器人带到碰撞位置

问题不在于“碰撞后如何脱困”，而在于全局规划和局部跟踪没有使用同一个安全契约：

1. `NavigationSystem::PathFootprintValid()` 只对离散 densified 路径点调用
   `costmap.lethal()`；
2. RPP 根据 carrot 和曲率生成连续圆弧，圆弧不等价于这些离散点的折线；在货架角点
   附近，圆弧会切入折线内侧；
3. RPP 的 `CollisionImminent()` 对 `linear == 0` 直接返回 `false`，没有验证当前
   footprint 仍处于安全距离；
4. 因此“全局路径点均安全”不等价于“RPP 实际执行轨迹安全”。机器人先被局部弧线
   推到障碍边缘，随后纯旋转使轮子空转、车体被接触约束，形成轮速非零而 IMU/刚体
   位姿不变。

这就是导致碰撞的导航 bug：**规划验证的是折线采样，执行的是未经同等 swept-footprint
验证的连续弧线**，并且原地旋转被错误地视为天然安全。

## 4. 修复方案

1. **导航输入去 oracle**：探索节点改用 Localization2D pose；速度使用轮速里程计，
   IMU 用于角速度校验。`/ground_truth` 订阅从控制节点移除。
2. **ROTATION_STALL 状态机**：命令非零但 IMU 与激光定位在短窗口内均无同向变化时，
   立即结束原地对齐，不等待整条路径超时。
3. **接触脱困**：从实时激光计算前后 clearance，生成 0.10--0.25 m 的 swept-footprint
   短平移候选；前进优先，只有前向不安全且后向安全时才允许短暂倒车，然后重新对齐。
4. **禁止重放失败首段**：按在线 map revision、当前位置栅格和路径前若干边生成
   `path_signature`；同一签名失败后临时加边惩罚，强制选择不同首段或同伦通道。
5. **补齐运行时诊断**：持久化 `/cmd_vel`、轮速、IMU、Localization2D pose、scan
   clearance、monitor action、TTC、map revision 和 path signature。Gazebo contact
   只作为模拟器诊断字段，不得作为导航决策输入。
6. **双层验收**：在线运行不暴露 truth；运行结束后独立脚本才使用 truth 栅格计算
   coverage 和使用 truth pose 计算 return error。在线 `COMPLETE` 不能替代离线门禁。

## 5. 实施与验证顺序

先增加非真值传感器记录和四类一致性单测，再实现 `ROTATION_STALL` 与候选脱困；随后
验证失败签名确实改变首段，最后重新并行运行 12 个 case。只有 12/12 同时满足离线
coverage、完成目标和返航门禁，才宣布修复完成。

当前工作区未按上述顺序实施，而是先改了第 3.1 节的规划/执行契约（未提交）：

- 探索节点规划器从 `theta_star` 换成 `state_lattice`，并加入 16 边形 footprint；
- `CollisionImminent()` 取消 `linear == 0` 豁免；碰撞时先二分缩小命令；
- state-lattice 路径不再叠半格膨胀半径；纯旋转/停车时挂起路径进展 watchdog。

第 4 节的去 oracle、`ROTATION_STALL`、接触脱困、`path_signature` 和运行时诊断都还未做。

## 6. 2026-09-08 未提交改动回归：5/12

用上述脏树重建 `sweepnav-simulation-desktop` 后，按同一矩阵跑完 12 个 case：

```text
EXPLORATION_TIMEOUT=1200 EXPLORATION_WALL_TIMEOUT=480 EXPLORATION_JOBS=3
SWEEPNAV_REBUILD=1 bash simulation/tools/run_exploration_matrix.sh
```

产物目录为 `simulation/artifacts/exploration/20260908T09*-<case>/`，完整日志
`simulation/artifacts/exploration/matrix-20260908T090548Z.log`。

| Case | 结果 | 覆盖率 | 目标数 | 返航误差 | 在线状态 / 现象 |
|---|---|---:|---:|---:|---|
| apartment-4 | 通过 | 0.9973 | 16 | 0.040 m | PARTIAL，进入返航并到家 |
| apartment-10 | 通过 | 0.9974 | 18 | 0.039 m | 同上 |
| maze-1 | 通过 | 1.0000 | 18 | 0.039 m | 同上 |
| maze-4 | 通过 | 1.0000 | 21 | 0.040 m | 同上 |
| maze-10 | 通过 | 1.0000 | 22 | 0.040 m | 基线失败，本次返航成功 |
| apartment-0 | 失败 | 0.9935 | 11 | 8.426 m | 探索超时，从未 `BeginReturn` |
| apartment-1 | 失败 | 0.9973 | 8 | 15.340 m | 同上 |
| warehouse-1 | 失败 | 0.9640 | 17 | 15.674 m | 同上 |
| warehouse-4 | 失败 | 0.9520 | 20 | 10.976 m | 同上 |
| warehouse-10 | 失败 | 1.0000 | 23 | 13.052 m | 覆盖完整，仍未进入返航 |
| warehouse-0 | 失败 | — | — | — | 无 `result.json`，探索中停滞 |
| maze-0 | 失败 | — | — | — | 无 `result.json`，开局 `start or goal is occupied` |

相对第 1 节 9/12 基线：只修好了 `maze-10`；`warehouse-0` / `maze-0` 仍失败且更差；
新失败是 `apartment-0/1` 和 `warehouse-1/4/10`。

## 6.1 新失败不是“返航执行失败”

离线门禁看到的是 coverage 达标 + `return_error > 0.10 m`，容易写成“返航超时”。
五个新失败的在线 `message` 都是 `exploration timed out`，日志里没有

```text
Final online-map return path accepted
returning to exploration start
```

通过的 `apartment-4/10` 则会在 frontier 耗尽后进入返航，最后目标变为 `(0, 0)`。
新失败从未调用 `BeginReturn`。`return_error_m` 只是超时瞬间到出生点的距离，不是
返航控制器的终点误差。

五个 case 的仿真跨度都约 116--120 s，正好用尽 `EXPLORATION_TIMEOUT=1200`
（10 倍速）的任务预算。

## 6.2 共同机制：可规划但不可执行的前沿把返航饿死

卡住后探索器仍能选出稳定 viewpoint，lattice 也常能给出全局路径，于是
`SelectNextGoal()` 不会走到“无可用前沿 → 安全返航”。实际执行在路径首段立刻失败，
再规划同一条或同类首段，循环到任务超时。

共同日志签名与第 1 节基线失败相同：

```text
requested=(0.000, +/-0.108)
published=(0.000, +/-0.108)
monitor_action=0
velocity=(0.000,0.000)
arc=0.00/<path_length>
planner_error=controller command produced no measured motion
```

| Case | 恢复次数 | 其中 arc=0 原地转 | dense footprint 拒路 | 卡住位姿（出现次数） | 反复目标 |
|---|---:|---:|---:|---|---|
| apartment-0 | 49 | 48 | 66 | `(7.25, -4.29)` ×43 | `(15.03, -4.62)` ×43 |
| apartment-1 | 110 | 108 | 4 | `(15.05, 2.94)` ×73 | 北侧一簇 viewpoint 轮换 |
| warehouse-1 | 60 | 45 | 26 | `(12.76, 5.10)` ×31 | 货架间多个目标 |
| warehouse-4 | 60 | 33 | 16 | `(-0.84, 6.56)` ×57 | 货架通道多个目标 |
| warehouse-10 | 20 | 18 | 6 | `(5.20, -8.45)` ×19 | 南侧残留 viewpoint |

`apartment-0` 最后 3 次 tour 仍是 `candidates=5 stable=5`；`warehouse-10` 覆盖率
已是 1.0，最后一次 tour 仍有 `candidates=1`。在线 frontier 图和离线 truth 覆盖
不是同一件事：残差 viewpoint 只要还能规划，就不会触发返航。

`CollisionImminent()` 取消原地转豁免后，这五个 case 的 `monitor_action` 仍为 0，
命令仍被发布。说明 costmap 并不认为当前 footprint 致死；卡的是接触/静摩擦下的
刚体零运动，不是栅格碰撞检查漏掉的 `linear==0` 分支。未提交的碰撞豁免修复没有
打到这条路径。

另一半失败是 `planned path failed dense footprint validation`。lattice 原语通过
后，`DensifyPath(..., 0.08)` 再按 `robot_radius` 做点采样检查；state-lattice
没有 Theta* 那条 A* 折线回退，拒路后只换下一个 viewpoint 或重放同一目标。

## 6.3 未提交改动如何把 9/12 打成 5/12

1. **`state_lattice` 首段是 SE(2) 原语，不是 Theta* 中心线。** 起点航向与第一段
   不一致时，RPP 合法输出 `v=0, ω=±0.108`。lattice 更频繁地要求原地对齐。
2. **旋转时挂起路径进展 watchdog** 后，唯一出口是 0.9 s 的 `CommandStalled`。
   监视仍把轮速/位姿增量当进展，接触空转时 pose 与 `/odom` 都是 0，于是
   `kBlocked` → 最新地图重规划 → 同一首段。没有 `path_signature`，43 次
   `apartment-0` 重放同一 `(pose, goal)`。
3. **costmap 看不见的接触** 让新的 `CollisionImminent()` 继续放行原地转；没有
   前后 clearance 脱困，底盘不会离开接触姿态。
4. **可规划残差前沿** 阻止 `BeginReturn`。覆盖率已经够，任务预算却耗在失败首段
   上，离线门禁就记成返航超差。

因此这五个“覆盖达标、返航超差”的直接原因是：**探索后半段在接触姿态上重放不可
执行的 lattice 首段，返航阶段根本没开始。** 要先让失败首段改变通道或触发脱困，
并在“反复不可执行”时强制进入返航；只改返航对接精度解决不了它们。

逐个修复建议顺序：`apartment-0`（单一 pose/goal 重放最干净）→ `apartment-1` →
`warehouse-4`（位姿锁定 57 次）→ `warehouse-1` → `warehouse-10`（覆盖已满，只剩
残差 viewpoint）。`warehouse-0` / `maze-0` 仍按第 1 节原失败处理。

## 7. apartment-0 修复方案

`apartment-0` 的停滞**已经被检测到**。`CommandStalled` 约 0.9 s 就给出
`kBlocked` + `controller command produced no measured motion`。坏的是恢复契约：
规划成功被当成“前沿可执行”，执行失败既不换首段也不进入返航。

本次只修这一条回放环，不把第 4 节六项一次做完。`CollisionImminent` 豁免和
`state_lattice` 都不是这个 case 的主修复。

### 7.1 现有代码为什么会重放 43 次

卡住点：`pose=(7.25, -4.29)`，目标 `(15.03, -4.62)`，首段
`requested=(0.000,-0.108)`，`arc=0.00/9.17`。

1. `ReplanActiveGoalOnLatestMap()` 用同一 `active_goal_` 重建
   `NavigationSystem`，没有首段约束，最多 2 次后又回到选点。
2. `Tick` 在执行 `kBlocked` 后明确**不** `RecordAttempt(false)` / 不进
   blacklist，只 `pop_front()`。注释把它当成“下一帧地图会给出新 viewpoint”。
3. `BuildTour()` 仍看到同一块 frontier，`stable=5`，同一目标回到队首。
4. `SelectNextGoal()` 里 `StartNavigation()` 成功就把
   `no_executable_frontier_cycles_` 清零。30 次强制返航只统计**规划前拒绝**，
   不统计**规划成功、首段空转**。

因此 lattice 能出路径 = 永远不会 `BeginReturn`。

### 7.2 两道门

**门 A：失败首段冷却，强制换通道。**

规划被接受时，用当前位姿栅格和路径前 1.0 m（或前 3 条 lattice 边 + 起始
yaw bin）生成 `path_signature`，写入 `NavigationState`。不要用整个路径或
只哈希 `(pose, goal)`：后者只会换目标，不会换同目标的另一条起步边。

当 `kBlocked` 且 `path_progress_m < 0.05` 时，把该签名放入
`(pose_cell → cooldown)`。下一次 `Replan` / `StartNavigation`：

- 新路径签名命中冷却则拒绝，原因写成 `failed first-segment replay`；
- lattice 对冷却边加临时代价，迫使起步原语换方向或换 yaw；
- 冷却随 `pose_cell` 改变或 `known_cells` 明显增长而失效，避免把门封死。

`ReplanActiveGoalOnLatestMap` 必须走这道检查；同一目标只有首段不同才允许重试。

**门 B：同一 pose 反复不可执行则强制返航。**

在探索节点增加与规划拒绝分开的计数，`StartNavigation` 成功不得清零：

- 同一 `pose_cell` 连续 3 次首段执行失败（含签名拒绝）→ `BeginReturn(true)`；
- 或 `execution_stall_cycles_ >= 6`（跨目标）且 pose 未离开当前栅格 → 同样强制返航。

执行 `kBlocked` 时对当前目标 `RecordAttempt(false)`，让 tour 不再立刻再生
同一 viewpoint。`apartment-0` 覆盖率已 0.993，强制返航满足“覆盖达标就该回家”。

### 7.3 脱困：否则回家首段还会转死

只做门 A/B，日志可以证明“换了签名 / 进入了 RETURNING”，但接触姿态下回家路径
仍可能是 `v=0,ω=±0.108`，返航误差还会是 8 m。`apartment-0` 要通过 0.10 m 门禁，
在下一次全局规划前加一步短平移：

- 用当前 `/scan` 算前后 clearance；
- 0.10--0.25 m swept-footprint 候选，前进优先；前向不安全且后向安全才倒车；
- 平移后 `pose_cell` 或 yaw 变化，门 A 的冷却自然让出新起步边。

这一步仍不读 `/ground_truth`。

### 7.4 改动位置

| 位置 | 改动 |
|---|---|
| `NavigationState` | 增加 `path_signature`、首段长度/起始 yaw |
| `NavigationSystem::Replan` | 生成签名；冷却边加代价；命中则 `failed first-segment replay` |
| `autonomous_explorer_node::Tick` | 首段 `kBlocked` 记冷却、`RecordAttempt(false)`、累加 stall 计数 |
| `ReplanActiveGoalOnLatestMap` | 禁止同签名重试 |
| `SelectNextGoal` | 规划成功不再清零执行失败计数 |
| `BeginReturn` | 由门 B 触发；返航失败再走一次脱困后重试，而不是立刻 `Finish` |

### 7.5 apartment-0 验收

单独跑 `apartment-0`（同 TIMEOUT/WALL）。通过条件：

1. 日志出现至少一次与上次不同的 `path_signature`，或明确
   `failed first-segment replay` 后改选其它 viewpoint / 起步边；
2. 同一 `(7.25, -4.29) → (15.03, -4.62)` 重放次数从 43 降到 ≤3；
3. 出现 `Final online-map return path accepted`；
4. 离线 `coverage_ratio >= 0.90`、`completed_goals > 0`、`return_error_m <= 0.10`。

只满足 1--3 而返航仍超差，说明门 A/B 已验证、缺的是 7.3 脱困，不要再回到
重放前沿。通过后再把同一契约套到 `apartment-1`。
