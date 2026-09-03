# Exploration 不完整 Case：问题分析与修复方案

## 1. 结论摘要

当前场景出现“不完整探索”的原因不是单一规划器故障，而是三个状态被错误串联：

```text
地图中仍有 unknown
  -> raw frontier 存在
  -> 没有可执行 viewpoint
  -> SelectGoals() 返回空
  -> 外层早期将其当作 frontier 耗尽
  -> 旧逻辑报告 COMPLETE
```

伪 `COMPLETE` 已通过状态机修复被拦截，但当前真实场景仍未达到全图覆盖。最新 Docker 矩阵中，12 个 case 没有一个满足严格通过条件：部分 case 返回 `PARTIAL`，部分 case 在最终返航阶段失败或超时。

## 2. 测试环境与结果

运行命令：

```bash
cd /home/jesse/workspace/sweepnav_2d
EXPLORATION_TIMEOUT=1200 \
EXPLORATION_WALL_TIMEOUT=480 \
EXPLORATION_JOBS=3 \
SWEEPNAV_REBUILD=1 \
bash simulation/tools/run_exploration_matrix.sh
```

场景为 apartment、warehouse、maze，各使用 seed `0/1/4/10`，共 12 个实际 Docker case。

最新矩阵的主要结果：

| Case | 结果 | 目标数 | 关键现象 |
|---|---|---:|---|
| apartment-0/1/4/10 | PARTIAL | 17–20 | raw frontier 存在但无法生成可执行入口 |
| warehouse-0 | FAILED | 20 | 最终返航超时，误差约 17.22 m |
| warehouse-1/4/10 | PARTIAL | 35–42 | 探索推进较多，但仍有未处理 frontier |
| maze-0 | 无 result | — | 进程未正常生成结果文件 |
| maze-1 | PARTIAL | 16 | 1 个 stale goal，仍有剩余 frontier |
| maze-4 | PARTIAL | 20 | raw frontier 仍被入口条件过滤 |
| maze-10 | FAILED | 21 | 最终返航执行失败，误差约 15.18 m |

注意：`known_cells` 不能作为覆盖率。它包含 occupied/free，且受动态地图窗口影响；最终通过必须使用 truth 栅格离线计算可探索自由空间覆盖率。

## 3. 已确认问题

### 3.1 raw frontier 与 executable goal 被混为一谈

`FrontierExplorer::SelectGoals()` 的处理链是：

1. 从 `free -> unknown` 邻接关系提取 raw frontier；
2. 按最小组件大小过滤；
3. 对候选 viewpoint 做 footprint clearance；
4. 做 BFS 可达性检查；
5. 做 standoff 和 LOS 检查；
6. 做稳定性观察和黑名单过滤。

只要第 2–6 步全部失败，函数就返回空列表。旧状态机无法区分“raw frontier 为零”和“raw frontier 存在但没有 executable goal”，因此会在 `empty_frontier_cycles_` 达到阈值后返航。

### 3.2 动态地图边界造成 frontier 漏检

地图是动态扩展的。地图当前窗口边缘仍可能是 unknown，但旧 frontier 扫描只遍历内部栅格，边界的 free/unknown 接触带无法生成任务。结果是 mapper 没有收到继续探索边缘的驱动，右侧未知区可能长期留在窗口外或边界附近。

### 3.3 窄道约束之间存在冲突

当前窄道策略已经允许实体半径 clearance，但统一的：

```text
minimum_standoff = 0.55 m
```

会拒绝 0.70 m 窄道中合法的观察姿态。窄道内机器人中心到未知边界通常只有 0.28–0.35 m，因此 footprint 允许并不代表 standoff 允许。

### 3.4 入口 LOS 过于严格

窄道入口附近可能被门框、墙角遮挡。安全入口 viewpoint 实际可用于“进入并继续扫描”，但当前目标生成要求从入口直接对 frontier cell 建立 LOS，导致入口任务在候选阶段被过滤。

### 3.5 原地探测恢复会消耗任务预算

为解决无候选入口问题加入过原地旋转扫描。初版每次扫描 6 s 且可重复触发，导致完整矩阵中的 12 个 case 都出现任务预算被恢复动作消耗、最终返航超时。后续已限制为最多 3 次、每次 2 s，但该策略仍需验证地图增长效果，不能作为无限重试循环。

### 3.6 最终返航阶段独立失败

warehouse-0、maze-10 等 case 已完成较多探索目标，但最终在线地图返航路径在控制执行阶段反复失败或超时，表现为：

- 全局返回路径被接受；
- RPP 持续输出控制；
- 局部控制器无法执行已验证路径；
- 两次在线地图重规划后仍无法收敛；
- 最终返航误差达到 10–18 m。

这不是 frontier 生成问题，需要单独修复返航策略。

### 3.7 结果状态与覆盖率验收仍需解耦

运行时没有场景 truth，不能在线计算真实 coverage。正确模型应为：

- 在线：只用传感器地图和 frontier 做保守收敛判断；
- 离线：读取最终 map/snapshot 与同次生成的 truth 栅格，计算真实可探索自由空间覆盖率。

## 4. 失败分类与判定规则

### A. 探索未收敛

特征：

- `raw_frontier_cells > 0`；
- `candidate_frontier_goals == 0`；
- 地图仍有 unknown 连通区；
- 不应报告 `COMPLETE`。

处理：继续入口恢复/边界探测；达到恢复预算后报告 `PARTIAL`。

### B. 真实目标不可达

特征：

- 多个独立 viewpoint 均规划失败；
- 地图多轮无增长；
- 入口被障碍或机器人 footprint 几何上封死。

处理：记录为 confirmed unreachable，不应无限重试。

### C. stale goal / 瞬态安全中断

特征：

- 地图刷新后 start/goal occupied；
- TTC、碰撞监视器或局部控制器短暂阻止；
- 后续地图更新后可重新生成任务。

处理：不计入 failed goal，不加入永久黑名单。

### D. 最终返航失败

特征：

- frontier 探索已基本停止；
- final return path 存在；
- 控制跟踪失败或超时；
- return error 大于验收阈值。

处理：进入专用返航恢复流程，不能把结果伪装成 COMPLETE。

## 5. 修复方案

### P0：修复状态语义和完成门禁

1. `SelectGoals()` 返回结构化诊断，而不是只返回 goal vector：
   - raw frontier cells/components；
   - 过滤原因计数；
   - candidate goals；
   - stable goals；
   - confirmed unreachable。
2. 外层状态机严格区分：
   - `NO_RAW_FRONTIER`；
   - `RAW_FRONTIER_BLOCKED`；
   - `CANDIDATE_WAITING_STABILITY`；
   - `EXPLORING`；
   - `RETURNING_PARTIAL`。
3. 只要 raw frontier 存在，就禁止进入 COMPLETE。
4. 运行时完成状态改为 `COMPLETE_CANDIDATE`；最终结果由离线 truth 覆盖率验证器决定。

### P1：动态地图边界和入口恢复

1. 对动态地图边界建立显式 boundary frontier：只有边界内侧 free cell 安全时，才生成靠内侧 viewpoint。
2. 当 raw frontier 存在但没有 viewpoint 时，执行有限次入口恢复：
   - 原地扫描；
   - 沿已知自由空间向边界前进一个短步长；
   - 在新地图 revision 后重新生成 frontier。
3. 恢复动作设总预算（次数、时间、地图增长量），超过预算后返回 PARTIAL。
4. 任何恢复动作都不能把 unknown 当作可通行空间。

### P1：专用窄道 viewpoint

1. 用 corridor detector 判定单向窄走廊，而不是通过全局参数放宽。
2. corridor 中同时满足：
   - 实体半径 clearance；
   - BFS 可达；
   - standoff `>= robot_radius`；
   - 与入口或 component member 至少一条可观测路径。
3. 若入口 LOS 被门框遮挡，允许先到入口安全侧，再进行旋转/短步扫描。
4. 开阔区域保持原 footprint 和 standoff 约束。

### P1：最终返航恢复

1. 返航路径执行失败时，区分“路径失效”和“控制跟踪停滞”。
2. 控制停滞时执行：减速、短距离反向脱困、重新定位、局部地图重规划。
3. 连续失败后切换到保守 A*/栅格路径，并降低路径曲率和速度。
4. 返航误差超过阈值时只能返回 FAILED，不能因为探索目标完成而返回 COMPLETE。

### P2：离线 truth 栅格验证器

1. world 生成器输出 truth occupancy 栅格及可达自由空间 mask。
2. 验证器将 snapshot/map 投影到统一分辨率和原点。
3. 只统计 truth 可探索自由栅格：

```text
coverage_ratio = observed_free_cells / reachable_free_cells
```

4. 同时输出：
   - unknown components；
   - largest unknown component；
   - 窄道区域观测率；
   - 地图/真值差分图。
5. 测试脚本只依据离线验证结果决定通过，`known_cells` 仅作为诊断字段。

已实现 `simulation/tools/validate_exploration_coverage.py`：它从同次运行的 `world.sdf` 解析
障碍物，以 5 cm 栅格和 0.28 m 机器人半径构造 truth 可达自由空间，并将最终 snapshot 投影
到 truth 坐标系，输出 `coverage.json`。`run_exploration_demo.sh` 现在会在状态检查前运行该
验证器；即使运行时错误上报 `COMPLETE`，coverage 小于 0.90 也会被拒绝。

验证器已修正坐标系：探索器将 `/ground_truth` 转换为“出生点为原点、初始 yaw 对齐”的局部
map frame，truth 栅格在比对 snapshot 前必须执行同样的 SE(2) 变换。此前直接用世界坐标查
snapshot 会把完整轨迹错误判为低覆盖（约 0.30）；修正后 maze-1/4/10 的覆盖率均为 1.000，
apartment 四个 seed 约 0.997，warehouse-1/4/10 为 1.000，warehouse-0 为 0.944。

在线状态可以保守返回 `PARTIAL`，但场景验收以离线 `coverage_ratio >= 0.90` 且
`return_error_m <= 0.10` 为准；测试脚本会写入 `validated_status=COMPLETE`，同时保留原始
在线状态用于诊断。

坐标修正后的最新实际矩阵（并行度 3）中，除一次并发运行下的 `maze-4` 早退外，其余
case 的离线 coverage 均达到门禁：apartment `0.9958–0.9973`，warehouse `1.000`，
maze-0/1/10 `1.000`。单独重建镜像运行 `maze-4` 已达到 `1.000`（77,541/77,541）且
返航误差 `0.039 m`；并行矩阵中的该次失败表现为仅覆盖 `0.752`、13 个目标后超时，
需继续定位并发 Gazebo/桥接启动或控制时序对该出生点的影响，不能据此宣称 12/12 已稳定通过。

随后对 `maze-4` 做了两次独立 Docker 复测（并行度 1、同一 seed、无需重建镜像），两次均为：
`coverage_ratio=1.000`（77,541/77,541）、返航误差约 `0.0395 m`，分别完成 26 和 23 个
frontier 目标。由此确认 `maze-4` 算法路径在独立运行下已稳定；并行矩阵中的偶发早退仍需
作为 Gazebo/ROS 并发隔离问题单独处理。

## 6. 推荐实现顺序

1. 先完成 P0 状态语义和日志，保证不会再伪 COMPLETE；
2. 加入动态边界 frontier 和有限入口恢复；
3. 完善 corridor viewpoint，覆盖 0.70 m 窄道及 L 形入口；
4. 单独修复 final return recovery；
5. 实现 truth 栅格离线验证器；
6. 重新运行 12 个 Docker case，再按 coverage_ratio 判定；
7. 最后才调整速度、超时等运行参数。

## 7. 验收标准

### 探索正确性

- 任意 raw frontier 存在时不得报告 COMPLETE；
- 右上窄道必须实际进入或完成近距离观测；
- 外包围内不存在大面积连续 unknown；
- 所有 case 的 truth `coverage_ratio >= 0.90`，目标 maze case 建议 `>= 0.95`；
- 窄道区域单独观测率不得低于全局阈值。

### 返航正确性

- 最终返航误差 `<= 0.10 m`；
- 返航控制失败不得被标记为 COMPLETE；
- 返航恢复有明确次数和时间上限。

### 测试正确性

- 12 个 Docker case 全部生成 result、snapshot、truth 和差分统计；
- 截断轨迹或屏蔽右侧激光的负例必须被覆盖率门禁拒绝；
- C++ 单元测试和 Docker 实际场景测试均通过后，才能宣布修复完成。

## 8. `requested/published != 0` 但实测速度为零：根因闭环

### 8.1 现象与排除项

失败产物 `20260903T083552Z-maze-4` 的关键日志为：

```text
pose=(8.87, 6.72) goal=(3.63, 7.83)
requested=(0.048,-0.108) published=(0.048,-0.108)
monitor_action=0 planner_error=none velocity=(0.000,0.000)
```

随后同一位姿、同一路径连续重规划，最终超时（coverage `0.7524`，return error
`11.1253 m`）。这不是规划器没有输出速度，也不是 ROS/Gazebo 桥接丢包：在同一 Docker
world 中用 `gz topic -e -t /cmd_vel` 可以看到该速度连续到达 Gazebo 的 DiffDrive 插件；
`/ground_truth` 和 `/odom` 在接触后同时保持零速度。

### 8.2 可重复的物理原因

`maze-4` 出生点为世界坐标 `(7.4, 5.5)`、初始 yaw 为 `pi`。将失败日志中的局部位姿转换回
世界坐标，机器人位于约 `(-1.47, -1.22)`，朝向约 `3.94 rad`。此时 RPP 输出
`(0.048 m/s, -0.108 rad/s)` 会沿圆弧向 `maze_wall_3` 靠近。该墙体右表面约在
`x=-2.09`，机体半径为 `0.28 m`；直接在 Gazebo 将机器人放到该位姿并持续发布同一命令，
机体稳定停在 `(-1.81, -1.37)`，恰好达到 `0.28 m` 实体接触边界，复现了失败轨迹中约
`0.17 m` 的位移后静止现象。

根因是安全链路的两个盲点叠加：

1. 激光安装在机体前方 `0.18 m`，LD14 的 `range_min=0.15 m`。机体贴墙时，墙距雷达仅
   `0.10 m`，前向扫描在 ROS 中变为 `-inf`（低于量程下限）。旧代码把所有非有限值直接
   丢弃，因而认为前方没有障碍；在该姿态实际采样到的前向 11 个 beam 全为 `-inf`。
2. 即使在进入盲区前，CollisionMonitor 只预测 `2.0 s`，RPP 只预测 `0.35 s`。在
   `0.048 m/s` 下分别只有约 `0.096 m` 和 `0.017 m` 的投影距离，且栅格碰撞检查使用
   与实体半径相同的 `0.28 m`，没有为栅格量化和连续圆弧留下余量。机器人因此可以从仍
   有约 `0.33 m` 中心净距的位置继续逼近到实体接触。

接触后速度为零是 Gazebo 物理结果，不是反馈假值；`wheel_odometry.py` 的 `/odom` twist
直接来自 `/ground_truth`，所以会如实报告零。规划器仍显示 `none` 是因为它只负责回答
“静态栅格上是否存在一条路径”，没有收到“执行器被实体接触卡住”的信息。

### 8.3 已实施的根因修复

本次修复没有加入倒车、随机旋转或切换目标等绕过动作，改动集中在安全和几何一致性：

1. **近距盲区方向性硬制动**：`CollisionMonitor::UpdateLaserScan()` 将负无穷和有限的
   sub-minimum return 记录为 `range_min` 处的盲区障碍点；若当前平移命令使机器人朝该点
   接近，输出新的 `kBlindZoneStop`，并保持触发/释放滞回。原地旋转或远离障碍不触发，
   不会破坏窄道内的姿态调整。
2. **规划与控制的几何契约统一**：全局 A*/Theta* 的连续路径验证使用
   `robot_radius + sqrt(0.5) * map_resolution`，把栅格单元半对角线作为离散化误差预算；
   RPP/MPC 的在线 swept-footprint 检查使用真实 `robot_radius`，避免把同一误差重复叠加后
   在合法窄道内永久停转。实体半径从不被缩小，0.70 m 窄道仍保留约 `0.07 m` 每侧几何余量。
3. **基于执行反馈的停滞判定**：NavigationSystem 在非零命令持续窗口内同时检查真实位姿
   位移、航迹弧长和航向变化，而不是只看单帧里程计速度。连续约 `0.9 s` 没有累计运动就
   记录 `controller command produced no measured motion`，清空当前路径并立即返回
   `kBlocked`；ROS 节点随后以最新在线地图和当前实测位姿重建规划。这个状态转移不发送
   固定倒车/旋转指令，也不把目标标记为不可达。
4. **里程计新鲜度约束**：ROS 节点记录 `/odom` 的消息时间戳，超过 `0.20 s` 未更新的
   速度样本不再参与加速度限制或停滞诊断，避免把上一段运动的旧速度当成当前执行反馈。
   当里程计短暂延迟时，累计位姿/路径进度仍用于判断真实底盘是否运动。
5. **回归保护**：新增用例验证负无穷前向回波会制动、远离命令可以释放、实体墙面接触会
   触发执行停滞，以及在 `0.315 m` 连续投影距离处会拒绝进一步逼近。

### 8.4 验证状态

本地 C++ 回归测试已通过 `8/8`。Docker 场景复测必须使用包含上述改动的 desktop 镜像；
验收仍按第 7 节的 truth 栅格覆盖率和返航误差执行。若后续日志再次出现
`published != 0`、`velocity == 0`，应同时检查 `monitor_action`、TTC、近距 beam 状态、
`controller_maneuver` 和真实位姿增量：有 `kBlindZoneStop` 是安全层制动；无制动但位姿也
不变则必须出现执行停滞原因并触发最新地图重规划；若位姿航向仍在变化，则属于正常对准
阶段，不应误判为物理卡死。

### 8.5 maze-0 时序复现与补充修复

在同一镜像的 maze-0 重跑中，曾观察到 RPP 连续输出 `(0,-0.108)`，日志采样的 odometry
为零，但机器人随后仍完成了姿态变化。这说明“单帧速度为零”也可能是对准阶段的采样时序，
不能直接把它当成碰撞。另一方面，撞墙时会长时间保持相同位姿，且路径弧长不增加。为区分
两者，停滞判定采用累计窗口：

```text
non-zero command
  + 0.9 s 内位姿平移 < 0.025 m
  + 0.9 s 内（纯旋转时）位姿 yaw/里程计旋转积分 < 0.03 rad，且目标航向误差无改善
  + 0.9 s 内路径弧长增长 < 0.02 m
  + 新鲜里程计积分平移 < 0.015 m 且积分旋转 < 0.03 rad
  => execution stall / kBlocked / latest-map replan
```

真实转弯或沿圆弧行驶会满足平移/航迹增长，纯原地对准会用实际 yaw 或旋转积分作为运动
证据，并辅以目标航向误差改善，不会被该判定打断；安全减速后的低速前进也会通过新鲜
里程计积分证明自身在运动。物理
接触造成的微小脉冲不会因为某一帧瞬时速度超过阈值而清除故障窗口，因为里程计先经过
时间戳新鲜度检查。该修复仍保持
碰撞监视器和实体 footprint 的硬约束，目的是把执行器实际状态反馈给规划闭环，而不是
绕过安全层。

### 8.6 最新实际回归

重建 desktop 镜像后使用同一 maze 出生点 `seed=4` 复测：

```text
run_id               = 20260903T105658Z-maze-4
reachable_free_cells = 77541
covered_free_cells   = 77541
coverage_ratio       = 1.000
unknown_free_cells   = 0
return_error_m       = 0.03977
completed_goals      = 20
no measured-motion recoveries = 0
```

该次没有出现非零命令/零位姿的执行停滞，右侧窄道及其后方区域均被 truth 栅格覆盖，
最终返航误差满足 `0.10 m` 门禁。在线结果仍可能是 `PARTIAL`，因为 mapper 在结束时
残留极小 frontier；脚本已将离线 truth 覆盖率与返航结果写入 `validated_status=COMPLETE`，
并保留原始在线状态用于诊断。此前 maze-0 的复现则证明：覆盖率可以达到 `1.000`，但
若最终返航控制超时，结果仍必须是 `FAILED`，不能用覆盖率掩盖执行失败。
