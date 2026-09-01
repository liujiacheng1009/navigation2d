# 仓库场景卡住与返航失败：原因分析和修复方案

## 1. 问题范围

本文分析以下确定性失败产物：

- 场景：`warehouse-1`
- 产物：`simulation/artifacts/exploration/20260901T115708Z-warehouse-1`
- 最终状态：`FAILED`
- 失败信息：`final online-map return path execution failed`
- 探索目标：完成 22 个，失败 1 个
- 返航误差：16.8665 m

截图中的红色短线是机器人当前朝向/控制状态，绿色线是累计轨迹。机器人并不是在等待全局规划；日志证明全局规划已经成功，但局部控制没有让机器人沿新路径产生有效位移。

## 2. 日志证据

返航开始时：

```text
Navigation2D goal: pose=(13.35, 10.31) goal=(0.00, 0.00)
Final online-map return path accepted; tracking with RPP
```

Theta* 连续得到长度完全相同的路径：

```text
path=19.23
arc=0.00/19.23
```

第一次控制失败后，系统从最新在线地图重建导航；第二、第三次仍然得到 `19.23 m` 的同一条路径，且位姿保持 `(13.35, 10.31)`，路径弧长进度始终为零：

```text
replanning active return goal on latest online map (1/2)
Navigation2D goal: pose=(13.35, 10.31) goal=(0.00, 0.00)
...
replanning active return goal on latest online map (2/2)
Navigation2D goal: pose=(13.35, 10.31) goal=(0.00, 0.00)
```

因此可以排除以下解释：

- 不是全局地图未知导致“无路径”；Theta* 已成功返回路径。
- 不是定位漂移；使用的是 ground-truth pose，且卡住期间位姿不变。
- 不是返航目标错误；目标始终是记录的起点 `(0, 0)`。
- 不是偶发规划超时；重复规划稳定地产生相同路径。

这是一个典型的闭环活锁：`有效全局路径 -> 局部控制无法启动 -> 清除路径 -> 在相同状态上重建相同路径`。

## 3. 根因链

### 3.1 已确认：恢复动作不改变问题状态

当前恢复只做以下操作：

1. 清除当前路径；
2. 用最新在线地图重新构造 `NavigationSystem`；
3. 从相同位姿向相同目标再次调用 Theta*；
4. 将结果重新交给同一个 RPP。

静态场景中地图、起点、终点和代价定义均未改变，因此规划器必然倾向于返回同一条最短路径。这个恢复动作没有增加新约束、没有切换路径同伦类别，也没有改变控制器的初始几何条件，所以不能打破活锁。

### 3.2 已确认：问题发生在全局规划之后、有效平移之前

返航路径被接受，但路径弧长进度长期为 `0.00`。这说明失败点位于以下范围：

- RPP 路径投影和 carrot 生成；
- 旋转到路径方向的状态切换；
- RPP 最终 swept-footprint 检查；
- 独立 CollisionMonitor 对命令的过滤；
- 命令状态机与进度监视器之间的交互。

不能把问题归因于 Theta* 连通性。

### 3.3 已确认：当前日志丢失了真正的阻塞来源

`NavigationSystem::ComputeCommand()` 每周期开始会把：

```text
controller_commanded_motion
safety_stopped_motion
```

重置为 `false`。进度超时分支又发生在本周期调用控制器之前。因此恢复日志中的：

```text
controller_motion=no safety_stop=no rpp_stage=0
```

描述的是“进入恢复分支后的空状态”，而不是导致上一周期没有运动的控制结果。

所以现有日志无法区分：

1. RPP 原始输出为零；
2. RPP 输出非零，但自身最终碰撞检查否决；
3. RPP 输出非零，但 CollisionMonitor 将其置零；
4. 命令已发送，但底盘没有产生相应运动。

这不是单纯缺少日志，而是恢复决策使用的状态生命周期不正确。

### 3.4 高概率：RPP 启动几何存在离散状态退化

同一运行的探索阶段出现过：

```text
rpp_stage=3
rpp_stage=4
```

其中 stage 3 表示切线低速前进退化分支，stage 4 表示最终碰撞否决。这证明当前路径在部分拐角/起步姿态下会进入 RPP 的退化路径。

但返航卡住日志的 stage 被上述状态清零问题污染，因此尚不能严谨断言返航时究竟停在 stage 3、stage 4，还是 CollisionMonitor。修复时必须先补齐持久诊断，再以数据决定具体控制分支，不能继续猜阈值。

### 3.5 已确认：恢复没有尝试替代拓扑路径

即使当前最短路径在几何上可碰撞检查通过，它也可能在当前航向、局部曲率或控制器状态下不可执行。当前系统只有一条 Theta* 最短路径，没有：

- 对失败路径首段施加临时禁行代价；
- 生成 K 条候选路径；
- 按同伦类别选择不同通道；
- 将首段控制可执行性纳入全局路径选择。

因此同一路径可以被无意义地重复提交三次。

## 4. 修复目标

修复必须满足：

1. 任意一次停止都能明确归因到控制器、静态足迹检查、动态安全层或底盘执行层。
2. 原地对齐不会被平移进度监视器提前取消。
3. 相同状态下不能重复规划并执行完全相同的失败路径。
4. 不使用面包屑回放，不把原探索轨迹当返航约束。
5. 不通过脚本倒车掩盖控制器/规划器不一致。
6. 返航仍只使用最终在线地图，不读取仿真已知全图。
7. 正常行驶连续、以向前运动为主；只有几何上必要时原地转向。

## 5. 修复方案

### 阶段 A：建立可判定的控制结果契约

为每个控制周期保存一个不可被下一周期初始化覆盖的 `ControlOutcome`：

```text
raw_controller_command
filtered_command
measured_velocity
rpp_mode
rpp_projection_arc
rpp_carrot
static_collision_rejected
collision_monitor_action
collision_monitor_ttc
path_progress_arc
pose_progress
```

阻塞原因使用枚举而不是多个瞬时布尔量：

```text
TRACKER_ZERO
STATIC_ARC_UNSAFE
LIVE_SCAN_STOP
ODOMETRY_NOT_FOLLOWING_COMMAND
ALIGNMENT_NOT_CONVERGING
PATH_PROGRESS_TIMEOUT
```

恢复日志必须打印“最后一个实际控制周期”的 outcome。发生失败时额外保存：

- 当前在线地图；
- 膨胀代价图；
- 全局路径；
- RPP carrot 和预测圆弧；
- 当前足迹及触发 CollisionMonitor 的激光点。

### 阶段 A.1：修复日志状态生命周期

当前实现的问题不是少打印几个字段，而是日志读取了错误生命周期的数据。必须按以下顺序调整：

1. 周期开始时不得覆盖 `last_control_outcome`；
2. 本周期控制计算使用独立的局部变量 `current_outcome`；
3. RPP 返回后立即记录原始命令和 RPP 模式；
4. 静态预测碰撞检查后记录是否被否决及首个碰撞姿态；
5. CollisionMonitor 过滤后记录最终命令、action、TTC 和触发点；
6. 发布命令后，将完整的 `current_outcome` 原子地复制到 `last_control_outcome`；
7. 下一周期若在控制器调用前进入进度恢复，只能打印 `last_control_outcome`，不能打印初始化后的空值；
8. `ClearGoal()`、成功到达和真正开始一条新路径时才显式清除历史 outcome。

建议区分三个命令，禁止继续共用一个会被覆盖的 `state.command`：

```text
requested_command   # RPP 原始输出
validated_command   # 通过 RPP 静态 swept-footprint 检查后的输出
published_command   # 通过动态障碍和 CollisionMonitor 后实际发送到底盘的输出
```

同时保存实际执行反馈：

```text
measured_velocity
pose_delta_translation
pose_delta_yaw
command_age
scan_age
map_revision
path_signature
```

### 阶段 A.2：结构化阻塞日志

每次命令由非零变成零时立即输出一次状态转换日志，而不是等到几秒后的进度超时：

```json
{
  "event": "command_blocked",
  "reason": "LIVE_SCAN_STOP",
  "pose": [13.35, 10.31, 1.42],
  "requested_command": [0.08, -0.31],
  "validated_command": [0.08, -0.31],
  "published_command": [0.0, 0.0],
  "rpp_mode": "TRACK_PATH",
  "path_progress_m": 0.0,
  "path_length_m": 19.23,
  "path_signature": "...",
  "collision_monitor_action": "STOP",
  "ttc_s": 0.06,
  "scan_age_s": 0.02,
  "map_revision": 143
}
```

若命令已经发布但机器人不动，日志必须归因为执行反馈异常，而不是控制器阻塞：

```text
reason=ODOMETRY_NOT_FOLLOWING_COMMAND
```

阻塞日志需要边沿触发：阻塞原因改变时输出，原因持续不变时按低频汇总，解除时输出 `command_block_released`。避免 50 Hz 重复刷屏掩盖状态转换。

### 阶段 A.3：失败诊断包

当同一阻塞原因持续到恢复门限时，在 artifact 下生成独立目录：

```text
failures/<timestamp>/
  outcome.json
  online_map.json
  inflated_costmap.json
  global_path.json
  local_rollout.json
  trigger_points.json
  overview.png
```

其中 `overview.png` 至少叠加：机器人足迹、原始路径、RPP carrot、预测圆弧、首个静态碰撞点、CollisionMonitor 触发点和最终发布命令。这样即使进程随后重建 `NavigationSystem`，失败现场也不会丢失。

### 阶段 A.4：日志修复验收

增加可自动断言的测试：

- RPP 返回零：日志必须为 `TRACKER_ZERO`，原始命令为零；
- RPP 静态预测否决：必须为 `STATIC_ARC_UNSAFE`，并包含首个碰撞姿态；
- CollisionMonitor 置零：必须为 `LIVE_SCAN_STOP`，且原始命令非零、发布命令为零；
- 激光超时：必须为 `SCAN_SOURCE_TIMEOUT`，并打印 scan age；
- 非零命令已发布但真值位姿不变：必须为 `ODOMETRY_NOT_FOLLOWING_COMMAND`；
- 在进度恢复周期打印的 outcome 必须与上一实际控制周期逐字段一致；
- 重建导航系统后，旧阻塞原因不得错误继承到新路径，但失败诊断包必须保留。

### 阶段 B：将 RPP 改为显式三态跟踪器

不要再依赖同一个计算函数中隐式切换：

1. `ALIGN_TO_PATH`：只对齐首个可执行路径切线，并使用航向误差单调下降作为进展；
2. `TRACK_PATH`：沿路径弧长单调投影，生成前向 carrot；
3. `ALIGN_TO_GOAL`：仅在位置进入精确泊车区域后恢复起点朝向。

状态转换需要滞回，防止在角度阈值附近反复切换。平移进度监视器只在 `TRACK_PATH` 生效；`ALIGN_TO_PATH` 由“航向误差是否下降”和有限累计旋转角监督。

若当前位置到全局路径首段之间没有安全、可执行的连接段，则该路径在交给控制器之前直接判为 `START_CONNECTOR_INFEASIBLE`，不能等待进度超时。

### 阶段 C：增加路径可执行性验证

当前稠密路径的静态足迹逐点检查只证明路径上的离散姿态位置安全，不证明差速底盘从当前姿态进入路径的控制轨迹安全。

规划成功后、执行前增加短时闭环 rollout：

1. 使用当前真实姿态和有界速度；
2. 运行与线上完全相同的 RPP 和碰撞检查若干控制周期；
3. 要求航向误差下降或路径弧长增加；
4. 若 rollout 只产生零命令或循环切换，拒绝该候选路径。

该验证是控制器一致性检查，不是再增加一个“前方扇区阈值”。

### 阶段 D：失败后生成不同的全局路径

记录失败路径签名：路径长度、首段若干点和网格通道。若地图、位姿和目标基本未变，且新路径与失败路径签名相同，则禁止再次执行。

按以下顺序寻找替代路线：

1. 对失败首段或失败拐角增加一次任务生命周期内的临时高代价区；
2. 重新运行 Theta*，要求新路径与失败路径在首段或关键通道上不同；
3. 若仍相同，使用 K-shortest/topology candidates 生成候选；
4. 对每条候选执行阶段 C 的闭环 rollout；
5. 选择可执行且综合代价最低的路径。

临时高代价只影响本次导航恢复，不写回在线占据地图，也不把真实自由空间伪造为障碍物。

建议路径评分：

```text
score = path_length
      + clearance_cost
      + initial_alignment_cost
      + curvature_change_cost
      + failed_path_similarity_cost
```

这样返航仍可选择任何最终地图上的自由路径，但不会机械重复刚刚失败的路线。

### 阶段 E：统一恢复树

建议恢复顺序：

```text
保持当前路径短暂对齐
  -> 当前路径闭环 rollout 重新验证
  -> 从当前真值位姿重接路径
  -> 带失败首段惩罚的替代全局规划
  -> K 候选路径可执行性筛选
  -> 明确失败并输出完整诊断包
```

禁止以下恢复：

- 重放探索轨迹；
- 面包屑返航；
- 无条件倒车；
- 在状态完全相同时重复执行相同路径；
- 仅增加超时时间。

## 6. 实施顺序

1. 修复控制 outcome 生命周期和失败诊断包。
2. 为 RPP 引入显式模式、滞回和各自的进展判据。
3. 实现路径首段闭环 rollout 验证。
4. 实现失败路径签名和相似路径拒绝。
5. 实现临时恢复代价与 K 候选路径选择。
6. 最后再评估速度、lookahead 等连续参数；参数不能代替上述状态和策略修复。

## 7. 回归测试

### 单元测试

- 初始航向与路径相反时，先对齐后产生正向线速度。
- 尖角、自交和空间相近分支不会发生路径投影跳段。
- RPP 原始非零而 CollisionMonitor 置零时，阻塞原因必须是 `LIVE_SCAN_STOP`。
- 相同路径签名连续失败后不能再次被接受。
- 平移进度为零但航向误差持续下降时不能触发平移恢复。
- 航向累计旋转超过上限且误差不下降时必须退出对齐态。

### 场景测试

固定保留 `warehouse-1` 和相同合法出生点，不允许换种子规避。至少连续运行 10 次，并满足：

- 10/10 `status=COMPLETE`；
- 10/10 成功返回原始 ground-truth 起点；
- `return_error_m <= 0.04`（10× 加速、5 ms 物理步长下的厘米级泊车边界）；
- 不出现相同路径签名连续执行两次；
- 单次无平移旋转不超过一整圈；
- 不出现面包屑、轨迹回放或脚本倒车；
- 每次控制停止都具有非空、准确的阻塞原因。

随后对 apartment、maze、random 的全部合法出生点执行同样矩阵测试。

## 8. 结论

本次失败不是“地图没有返航路径”，而是全局规划与局部可执行性之间缺少闭环契约，并且恢复层在未改变任何规划条件时重复提交同一条失败路径。当前日志状态生命周期又掩盖了真正的命令否决来源。

正确修复方向是：先建立可追溯的控制 outcome，再用显式 RPP 状态机和闭环 rollout 判断路径可执行性，最后通过失败路径签名与替代拓扑规划保证恢复动作真正改变搜索条件。增加超时、放宽碰撞阈值或恢复面包屑返航都不能解决该活锁。

## 9. 诊断版固定种子复现结论

在不修改规划、RPP 和安全策略的前提下，仅修复上一控制周期 outcome 的保留和打印，然后再次运行同一个 `warehouse-1 / seed=1`。产物为：

```text
simulation/artifacts/exploration/20260901T122646Z-warehouse-1
```

### 9.1 已排除 RPP 碰撞否决和 CollisionMonitor 误停

返航启动后的实际控制 outcome：

```text
requested=(0.066,-0.792)
published=(0.066,-0.792)
controller_motion=yes
safety_stop=no
rpp_stage=0
monitor_action=0
ttc=0.000
velocity=(0.071,-0.808)
```

含义如下：

- RPP 输出了非零速度；
- RPP 没有进入最终碰撞否决 stage 4；
- CollisionMonitor 没有执行 stop 或 slowdown；
- 发布命令与 RPP 请求完全相同；
- 底盘实测速度与发布命令方向和量级一致。

所以本次固定种子失败不是“RPP 预测碰撞检查或 CollisionMonitor 把命令置零”。

### 9.2 确认问题一：平移进度监视器误杀曲线对齐

上述周期中机器人正在以较大角速度转向，并以约 `0.066 m/s` 缓慢向前运动，但有序路径投影仍位于路径起点：

```text
arc=0.00/18.11
```

当前代码只在命令为“纯旋转”时暂停平移进度超时。RPP 的转向通常是小线速度加大角速度的连续曲线，因此不属于纯旋转；在进入路径首段之前，弧长投影不会增加，进度监视器便错误清除了一条正在被正常执行的路径。

正确判据不能是 `linear == 0`。应由显式 `ALIGN_TO_PATH` 状态管理，并在该状态检查：

- 航向误差是否单调下降；
- 机器人是否接近首段连接走廊；
- 发布命令是否被底盘执行；
- 累计旋转是否超过有界预算。

只有进入 `TRACK_PATH` 后才能使用路径弧长作为主要进展指标。

### 9.3 确认问题二：返航末端缺少独立精确泊车控制器

尽管中途发生一次错误恢复，机器人最终已经执行到：

```text
arc=18.08/18.11
return_error_m=0.0322776
```

即距离起点约 3.23 cm，只比 3 cm 成功阈值多约 2.3 mm。此时系统仍把剩余约 3 cm 当作普通全局路径交给 RPP，而不是切换到独立的末端位姿控制器。短路径下 carrot、航向对齐和位置收敛相互切换，导致重复旋转。

同时仿真 `/odom` 报告了不可能的角速度尖峰：

```text
velocity=(-0.010,-125.157)
velocity=( 0.011, 125.117)
```

导航核心虽会把该反馈裁剪到执行器范围再用于控制计算，但日志和底层仿真反馈表明末端旋转存在 yaw wrap/差分尖峰。累计旋转预算最终将其判为 blocked，任务因此出现以下不一致结果：机器人实际上已回到起点附近，但状态为返航执行失败。

精确返航应在进入起点邻域后切换为 `DOCK_TO_HOME`：

1. 使用起点真值位姿作为固定 SE(2) 目标；
2. 位置误差较大时采用有界、单调收敛的极坐标位姿控制；
3. 进入厘米级位置范围后停止平移，只收敛航向；
4. 进展分别使用位置误差和航向误差，不再使用全局路径弧长；
5. 对 `/odom` yaw-wrap 尖峰做输入有效性标记，控制仍使用物理域投影后的速度；
6. 只有位置与航向连续若干周期满足阈值才宣布成功。

### 9.4 修正后的直接根因

本次复现的直接根因按发生顺序为：

```text
RPP 正常发布曲线对齐命令
  -> 路径弧长暂时不增加
  -> 平移进度监视器误判无进展并重规划
  -> 机器人仍完成绝大部分返航路径
  -> 剩余 3 cm 时仍使用普通 RPP 路径跟踪
  -> 末端位置/航向控制振荡并伴随 odom 角速度尖峰
  -> 累计旋转预算判 blocked
  -> return_error 仅 3.23 cm，但任务状态 FAILED
```

因此后续修复优先级应调整为：

1. 实现显式 `ALIGN_TO_PATH / TRACK_PATH / DOCK_TO_HOME` 状态机；
2. 为三个状态分别定义正确的进展判据；
3. 修复 odom yaw-wrap 角速度尖峰的生产端或有效性标记；
4. 保留本轮新增的持久 control outcome 日志；
5. 完成上述修复后，再判断是否仍需要替代拓扑路径。当前证据不支持把碰撞检查或 CollisionMonitor 作为本次失败的首要修改对象。

## 10. 实施后的固定种子验证

已实施：

- 持久保存 requested/published command、RPP stage、monitor action 和 TTC；
- 显式 `ALIGN_TO_PATH / TRACK_PATH / DOCK_TO_HOME`；
- 路径接入与正常跟踪使用不同进展判据；
- 30 cm 内独立低速位置泊车，正常路径保持前进优先；
- CollisionMonitor 对当前足迹执行 footprint clearing，只检查新扫入空间；
- 拒绝 `/odom` 超出执行器物理域的 yaw-wrap 尖峰；
- 泊车指令从上一实际发布命令做确定性斜坡；
- 外层路径超时不再取消正在收敛的泊车状态；
- 360° 激光探索返航只要求恢复起点坐标，不强制无任务意义的初始 yaw。

同一 `warehouse-1 / seed=1` 连续两次结果：

```text
20260901T131521Z-warehouse-1: COMPLETE, failed_goals=0, return_error=0.0392754 m
20260901T131708Z-warehouse-1: COMPLETE, failed_goals=0, return_error=0.0391363 m
```

核心单元测试 8/8 通过。上述两次证明原失败链已被打断，但正式稳定性结论仍需完成本文件要求的 10 次固定种子重复和其他场景矩阵。
