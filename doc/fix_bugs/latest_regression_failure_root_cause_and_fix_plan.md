# 最新 Docker 回归失败：根因分析与修复方案

## 1. 分析范围与验收口径

本次采用单 worker 顺序运行，避免多实例 Gazebo 争用 host `DISPLAY`、GPU 和
`/clock` 的启动竞态。覆盖率和返航均按离线 truth 栅格验收：

```text
coverage_ratio >= 0.90
return_error_m <= 0.10 m
completed_goals > 0
```

本轮结果为 7/12 通过、5/12 失败。`result.json` 中的在线 `PARTIAL` 只有在
truth 栅格和返航门禁都通过后才会被脚本写成 `validated_status=COMPLETE`。

| Case | 覆盖率 | 返航误差 | 结果 | 主要证据 |
|---|---:|---:|---|---|
| warehouse-1 | 0.8826 | 9.2895 m | 失败 | 前沿路径反复输出旋转命令，弧长为 0，最终超时 |
| warehouse-4 | 1.0000 | 18.4267 m | 失败 | 覆盖完成后返航首段持续旋转、位姿不变 |
| maze-0 | 0.6929 | 4.9876 m | 失败 | 前沿目标路径重复超时，右侧未知区域没有被进入 |
| maze-1 | 0.9943 | 13.7220 m | 失败 | stale goal 多次 `start or goal is occupied`，返航又发生零位移 |
| maze-10 | 1.0000 | 16.6857 m | 失败 | 覆盖完成，返航路径首段无位移并最终超时 |

代表产物：

- `simulation/artifacts/exploration/20260903T130641Z-warehouse-1`
- `simulation/artifacts/exploration/20260903T130858Z-warehouse-4`
- `simulation/artifacts/exploration/20260903T131325Z-maze-0`
- `simulation/artifacts/exploration/20260903T131545Z-maze-1`
- `simulation/artifacts/exploration/20260903T132807Z-maze-10`

## 2. 失败分类

### 2.1 前沿探索阶段执行停滞：warehouse-1、maze-0

`warehouse-1` 在已经完成多个前沿目标后，导航从约
`(14.86, 2.45)` 继续前往 `(0.63, 8.13)`，全局路径长度约 `19.3 m`。随后多次
出现：

```text
arc=0.00/19.32
requested=(0.000,-0.108) published=(0.000,-0.108)
monitor_action=0 ttc=0.000 velocity=(0.000,0.000)
```

控制命令并未被 CollisionMonitor 否决，但实测位姿没有沿路径前进。该路径在同一
起点被重复提交，直到单目标超时，最终覆盖率只有 `0.8826`。

`maze-0` 的模式相同：从 `(2.32, 9.06)` 前往 `(9.73, 4.33)` 时，路径长度约
`19.6 m`、弧长进度长期为零，重复超时后才切换到下一个目标，覆盖率降到
`0.6929`。这不是 truth 覆盖率计算错误，而是机器人没有穿过当前路径对应的
窄门/拐角。

这里的“纯旋转”不能直接等同于碰撞：在正常路径对齐时，yaw 可能正在变化而
平移尚未发生。但本轮日志没有保存每周期 yaw 增量、RPP carrot 和静态碰撞否决
原因，因而无法在失败发生的第一周期区分：

1. 正常对齐；
2. RPP 进入纯旋转极限环；
3. 静态 swept-footprint 拒绝平移；
4. 命令已发布但 Gazebo 接触约束使底盘没有实际运动。

### 2.2 最终返航执行停滞：warehouse-4、maze-1、maze-10

这些 case 的 truth 覆盖率已经达到或接近完整，失败发生在返回起点的控制阶段，
不是前沿覆盖不足。

共同日志形态是：

```text
Final online-map return path accepted
arc=0.00/<path_length>
requested=(0.000, +/-0.108) published=(0.000, +/-0.108)
monitor_action=0 ttc=0.000 velocity=(0.000,0.000)
```

当前返航恢复只做“清路径 -> 用最新地图重建 NavigationSystem -> 再次调用同一个
Theta* + RPP”。静态地图、起点、终点和当前位姿都没有实质改变，因此新规划很容易
再次得到同一条首段路径。`maze-1` 已经触发了
`controller command produced no measured motion`，但两次 latest-map retry 仍然
重放同一几何条件，最后返航误差为 `13.722 m`。

### 2.3 stale viewpoint：maze-1

`maze-1` 最后阶段保持：

```text
raw_cells=131 raw_clusters=31 candidates=1 stable=1
Navigation2D rejected goal before execution: start or goal is occupied
```

同一个 candidate 在没有有效地图增长的情况下被连续重建和拒绝。该目标已经不再
与当前在线地图的 safe footprint 契约一致，但它没有被按“当前 map revision 的
stale goal”冷却，导致选择循环消耗探索预算。此类拒绝不应计为不可达，也不应立即
触发返航；应等待新扫描或从同一 frontier component 生成下一个 viewpoint。

### 2.4 raw frontier 有但 candidate 为零：warehouse-4、maze-10

`warehouse-4` 出现 `raw_cells≈111、raw_clusters≈70、candidates=0`，随后
`maze-10` 出现 `raw_cells=79、raw_clusters=38、candidates=0`。这些 raw frontier
主要是地图边缘、墙背面或只有 1--5 个栅格的边界碎片，当前 `SafeViewpoint`、
standoff、BFS 和 LOS 过滤后没有合法 viewpoint。当前代码以固定次数计数：

```text
no_executable_frontier_cycles >= 30 -> BeginReturn(true)
```

计数由每秒一次的选择定时器驱动，在仿真加速下实际等待时间很短；同时没有记录
“被哪一层过滤”的原因。因此它既可能过早放弃一个尚未被新扫描打开的入口，也可能
一直对地图外侧的无效碎片做无效 BuildTour。该问题是探索状态机和 frontier
语义问题，不应通过降低全局安全距离解决。

### 2.5 并发测试基础设施竞态

并发度 3 的早期轮次中，某些 worker 的 Gazebo 进程启动但没有发布有效 `/clock`，
`docker compose exec` 内部的 `ros2 topic echo --once` 也可能不退出；另有多个
容器共用 `DISPLAY=:1`，产生 Xvfb/Openbox 监听冲突。这会造成“没有 result 文件”或
启动阶段假失败，必须与算法失败分开统计。

## 3. 根因链

```text
静态路径通过栅格检查
  -> 首段控制要求原地旋转或小曲率前进
  -> 实际底盘没有产生足够的平移/yaw 变化
  -> 当前恢复只重放同一条路径
  -> latest-map retry 次数耗尽
  -> 前沿覆盖不足或最终返航失败
```

当前系统缺少三个关键闭环：

1. **控制可执行性闭环**：全局路径只验证静态 footprint，没有在提交前验证 RPP
   从当前姿态出发能产生实际运动；
2. **失败多样性闭环**：恢复没有记录失败路径签名，也没有禁止同一路径首段被重复提交；
3. **frontier 语义闭环**：raw frontier、暂时没有 viewpoint、地图外边界碎片和
   确认不可达区域没有结构化区分。

## 4. 修复方案

### P0：先补齐控制结果和失败现场

增加持久化的 `ControlOutcome`，不能在下一周期开始时被清零：

```text
raw_controller_command
validated_command
published_command
measured_velocity
pose_delta_translation
pose_delta_yaw
path_progress_arc
rpp_mode / fallback_level / carrot
static_collision_reason
collision_monitor_action / ttc
scan_age / odom_age / map_revision
path_signature
```

每次“非零命令 -> 实际零位移”边沿立即记录 `command_blocked`；连续相同原因只做
低频汇总，解除时记录 `command_block_released`。达到恢复门限时保存：

```text
failures/<timestamp>/outcome.json
failures/<timestamp>/online_map.json
failures/<timestamp>/inflated_costmap.json
failures/<timestamp>/global_path.json
failures/<timestamp>/local_rollout.json
failures/<timestamp>/trigger_points.json
failures/<timestamp>/overview.png
```

这样可直接确认 warehouse-1/maze-0 是对齐极限环、静态碰撞否决还是 Gazebo 接触，
不再依赖最终 timeout 日志反推。

### P0：把纯旋转停滞变成可判定的状态机

新增独立状态 `ROTATION_ALIGNMENT` 和 `ROTATION_STALL`：

1. 只有 RPP 显式要求纯旋转，且当前 yaw 误差在下降时，才进入 `ROTATION_ALIGNMENT`；
2. 在短窗口内同时检查真实 yaw 增量、fresh odom 角速度积分和 heading-error 改善；
3. 若命令持续非零但三者均不足，立即进入 `ROTATION_STALL`，不能等待数个
   `progress_timeout` 周期；
4. 若存在可验证的平移方向，按 live lidar clearance 梯度生成短距离脱困候选，
   每个候选都经过 swept-footprint、TTC 和当前地图可达性检查；
5. 若没有安全脱困候选，给当前路径首段加临时 edge/voxel 禁行标记并生成替代路径，
   不再重放同一首段。

这里的脱困不是固定倒车脚本：方向由当前扫描、足迹和路径几何共同决定；只有在
前向不可行且后向 swept-footprint 安全时才允许短暂后退，完成后立即回到正常全局
规划。这样既保留雷达近距离可靠性，也不会用调参掩盖实体接触。

### P1：控制器感知的多候选路径选择

对 frontier 和 return 统一使用候选路径集，而不是单条 Theta* 路径：

1. 生成 Theta*、corner-safe A* 以及带临时失败边惩罚的替代路径；
2. 用当前 pose、当前 twist 和 live scan 对每条路径做 1--2 s RPP dry rollout；
3. 过滤首段无法产生平移/yaw 进展、静态 swept-footprint 不安全或 TTC 不满足的路径；
4. 对路径的 cell 序列/拓扑边计算 `path_signature`，失败路径在当前
   `(map_revision, stuck_pose_cell)` 下冷却，至少换一个首段或同伦通道；
5. 只有候选集为空时才报告 confirmed blocked，并将原因暴露给 ROS 节点。

返航必须复用该逻辑。不能因为最终地图有路径就认定返航可执行，也不能把同一条
20 m 路径重建三次当作恢复。

### P1：专用返航策略

返航不再走普通 frontier 的错误处理分支，建立独立的 `RETURN_ESCAPE`/`RETURN_TRACK`
状态：

1. 从最终位姿附近搜索一圈安全起步 pose，选择能使 RPP 首段前进的方向；
2. 先解决局部实体接触或窄门出口，再提交完整 home path；
3. 返航路径末端使用 home funnel/docking servo，只要求位置误差 `<=0.10 m`，不为
   任意初始 yaw 强制额外旋转；
4. 返航连续两次首段失败时切换到不同拓扑候选，而不是仅刷新同一地图；
5. 返航超过预算必须 `FAILED`，但保存完整 failure package，不能改写成 `PARTIAL`
   或 `COMPLETE`。

### P1：修复 frontier 状态机

将 `SelectGoals()` 改为结构化返回：

```text
raw_frontier_cells/components
candidate_goals
stable_goals
filtered_by_footprint
filtered_by_bfs
filtered_by_standoff
filtered_by_los
filtered_by_blacklist
boundary_only_components
```

外层按 map revision 和仿真时间处理：

1. `raw_frontier == 0` 才能进入 completion confirmation；
2. `raw_frontier > 0 && candidate == 0` 进入 `FRONTIER_ENTRY_PROBE`，等待地图增长或
   执行有限的 clearance-guided 入口探测；
3. 每次 `start or goal occupied` 只冷却该 goal/component，禁止同 revision 立即重试；
4. 只有在“地图 revision 在时间窗内无增长 + 入口探测预算耗尽 + 组件被证实位于
   地图外边界/不可达”时，才能把该组件标成 `CONFIRMED_UNREACHABLE`；
5. 未确认的 raw frontier 不得触发成功状态，只能等待、继续入口探测或返回
   `PARTIAL`。

固定 `30` 次 tick 不应作为唯一门限；应使用仿真时间、map revision 和 known-free
   增长量的联合门限，避免加速仿真下过早返航。

### P2：隔离并发测试基础设施

1. 每个 Compose worker 使用独立 `DISPLAY` 或关闭 RViz/Xvfb，只保留 Gazebo server、
   bridge 和测试节点；
2. `/clock` 探针使用可强制杀死的 timeout（例如 `timeout --kill-after=1 2 ...`），
   `docker compose exec` 也要有外层超时；
3. 先判断 worker 是否发布有效 clock sample，再启动 mapper/explorer；
4. 回归结果单独标注 `INFRASTRUCTURE_STARTUP_FAILURE`，不能混入算法失败率；
5. 算法验收先以顺序 12-case 为主，再用并发 3 做稳定性回归。

## 5. 实施顺序

1. P0：实现持久化 `ControlOutcome`、yaw/pose 增量和失败现场包；
2. P0：加入 `ROTATION_ALIGNMENT/ROTATION_STALL`，验证不再等待数百秒 timeout；
3. P1：加入路径签名、失败首段冷却和控制器 dry rollout；
4. P1：实现 clearance-guided 的几何脱困和专用返航状态机；
5. P1：结构化 frontier 过滤原因、stale goal 冷却和 map-revision 等待；
6. P2：修复并发 Compose 的 DISPLAY、clock 探针和 worker 状态分类；
7. 运行 C++ 单测、12 个顺序 Docker case，再运行并发矩阵；
8. 只有 12/12 同时满足 truth coverage 和返航门禁，才宣布探索完成。

## 6. 验收用例

至少增加以下自动化检查：

- 纯旋转命令、yaw 不变、实测速度为零：`ROTATION_STALL` 在短窗口内触发；
- 纯旋转命令、yaw 持续改善：不触发停滞，最终进入 tracking；
- 失败路径再次规划：`path_signature` 必须不同或首段临时禁行生效；
- `start or goal occupied`：同一 map revision 不重复尝试同一个 goal；
- raw frontier 存在但 candidate 为零：等待 map revision/入口探测，不得立即 COMPLETE；
- live lidar 盲区命中：向障碍运动被 `kBlindZoneStop` 制动，远离方向可释放；
- 返航覆盖率 100% 但位置误差大：结果必须为 FAILED；
- 并发 worker 无 `/clock`：在有界时间内归类为基础设施失败并清理容器。

## 7. 策略纠正：倒车应是受约束的后备动作

当前回归代码在 `autonomous_explorer_node.cc` 中设置：

```cpp
config.max_reverse_velocity = 0.;
```

这与实际任务目标不一致。探索和返航应当**前进优先**，但不能把倒车能力全局关闭：
机器人可能已经沿前进方向进入窄道或盲端，回程存在可行路径，却需要沿已走过的安全
轨迹短距离后退，或先后退到有足够转向净空的位置。

这里需要区分三个概念：

1. **偏好**：规划代价中对倒车增加惩罚，正常宽阔区域优先选择前进和转向；
2. **许可**：只有倒车候选通过当前 footprint、live lidar、swept-footprint 和 TTC
   检查时才允许执行；
3. **触发**：前向首段持续无平移进展、原地转向没有足够净空，或返航需要回溯已验证
   路径时，才启用倒车后备，不应把倒车作为每个目标的默认动作。

因此修复不能只把 `max_reverse_velocity` 改成非零，也不能只降低某个速度阈值。RPP
需要支持带符号的速度跟踪，返航/脱困状态机需要在“前进候选、可转向位置、倒车候选”
之间按安全性和路径代价排序，并使用滞回避免前进/倒车来回振荡。失败的倒车候选仍应
被记录为路径/边失败，交给下一轮候选路径切换，而不是无限重试同一首段。

本节替代此前“探索跟踪 forward-only”的表述；此前的全局禁用倒车属于过度收紧，正是
当前“已经走过来但返航第一段原地旋转停滞”问题的控制约束缺口。
