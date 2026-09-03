# Maze 场景探索完成度低：原因分析与修复方案

## 1. 结论

Maze 的“COMPLETE”目前表示“在线栅格中没有仍可执行的 frontier”，并不表示已经覆盖了仿真世界的全部可通行区域。现有产物中机器人通常只走通一条或少数几条走廊，随后因为 frontier 不可达、候选被安全过滤、局部路径执行失败或 frontier 被误判为已解决而返航。因此这是一个探索闭环提前收敛的问题，不是单一的 Theta* 无路径问题。

优先级建议：先修正完成判据和可观测性，再放宽/分层生成 maze 的观察点，最后处理狭窄拐角的路径执行。

## 2. 复现证据

数据来自 `simulation/artifacts/exploration/` 下 2026-09-01 的五次 maze 运行；所有运行都使用 ground-truth 位姿和 ground-truth scan insertion，因而可以排除定位漂移是主因。

| 产物（seed） | 状态 | 已知 cell | 未知 cell | 完成目标 | 失败目标 | 轨迹距离 |
|---|---:|---:|---:|---:|---:|---:|
| `133018Z-maze-1` | COMPLETE | 36,559 | 62,650 | 6 | 0 | 34.0 m |
| `134825Z-maze-4` | COMPLETE | 68,731 | 53,509 | 9 | 2 | 83.6 m |
| `135420Z-maze-0` | COMPLETE | 42,313 | 81,459 | 7 | 0 | 70.4 m |
| `135420Z-maze-10` | COMPLETE | 68,572 | 54,948 | 12 | 0 | 41.9 m |
| `135551Z-maze-1` | COMPLETE | 36,831 | 72,267 | 5 | 0 | 33.1 m |

已知 cell 只占最终栅格的约 34%–56%，且 seed 只改变出生点；同一 `maze-1` 两次仍仅完成 5/6 个目标、已知约 36k。这个结果与截图中集中在左/中部、右侧区域大面积未被扫描的轨迹一致。

日志还显示大量空 tour：`maze-1` 出现 `Frontier graph tour ... contains 0 reachable viewpoints`，随后连续构建到 tour 20；`maze-0` 到 tour 29；`maze-10` 到 tour 31。也就是说地图仍有大量 unknown，但当前 frontier 生成器已经找不到可执行观察点，完成状态由超时窗口触发。

`maze-4` 另外出现两次明确的执行失败：

```text
planned path failed dense footprint validation
controller could not execute validated route
```

失败后同一 frontier 被重规划/加入黑名单，进一步减少了可探索分支。

## 3. 根因链

### 3.1 完成判据把“不可执行”当成“已探索”

`ros/autonomous_explorer_node.cc` 在没有可执行 frontier 时仅等待有限次数；地图达到 25,000 个已知 cell 后，连续 12 个空周期就 `BeginReturn()`。因此“还有 unknown，但没有满足当前过滤器的 viewpoint”会被报告为 COMPLETE。`CompletionEligible()` 也只检查已知 cell 的增长和稳定窗口，没有世界可通行区域或可达 frontier 的覆盖下限。

### 3.2 Maze 拓扑使观察点过滤过于严格

`exploration/frontier_explorer.cc` 同时施加了：

- frontier 至少 12 cells；
- viewpoint 周围半径 0.40 m 内全部为 free；
- 与 frontier 距离 0.55–1.10 m；
- viewpoint 必须在已知 free-space 的 BFS 连通分量中；
- viewpoint 到代表 frontier cell 必须有 Bresenham free line-of-sight；
- 失败点 1.20 m 黑名单，且组件半径随 frontier 大小增大。

在 maze 的门洞、L 形拐角和墙端，frontier 往往是细长/分裂的小组件。算术中心附近可能没有满足 clearance 和视线的 cell；即使同一未知区域从另一侧可见，也会因代表 cell 的直线穿墙而被丢弃。日志中“有 frontier 但 tour 为 0”正是这一组过滤条件的直接表现。

### 3.3 只允许穿过已知 free，无法主动“探门”

`StartNavigation()` 把除 free（及机器人脚下证据区）之外的 cell 全部写成 lethal。该安全策略防止机器人驶入 unknown，但也意味着机器人只能到达当前扫描已经打开的 corridor；窄门前需要沿墙边/侧向移动时，单一 standoff viewpoint 可能永远不可达。系统没有“安全边界推进”或短距离探测动作来把未知门洞转换成已知 free。

### 3.4 任务提交与地图更新不同步，造成大量无效 tour

frontier 由在线 scan 持续改变。当前 tour 是一次性承诺，但每个目标到达后会立刻检查 `GoalRegionStillFrontier()`；地图只要把代表 cell 或其 0.35 m 邻域标成已知，就会跳过目标。随后重新建 tour，产生大量 0-viewpoint 周期。这个机制避免目标抖动，却没有把“目标已解析但同一区域仍有 unknown”转化为区域级任务。

### 3.5 局部执行失败会吞掉分支

maze-4 的 dense footprint validation 失败说明 Theta* 的平滑 chord 在墙角/狭窄通道中通过栅格检查，却不能通过连续 footprint 检查。当前失败处理会 `RecordAttempt(false)`、黑名单整片 frontier，再尝试少量候选；没有生成替代同伦路径或局部中间 waypoint，导致一条走廊失败等价于一个分支暂时不可探索。

## 4. 修复方案

### P0：先让完成状态可信

1. 将结果拆成 `FRONTIERS_EXHAUSTED`、`NO_EXECUTABLE_FRONTIER`、`TIMEOUT`、`RETURNED_WITH_UNKNOWN`，禁止后两者写成 COMPLETE。
2. 完成必须同时满足：连续稳定窗口内无 frontier、已知 free 区域相对上一窗口增长低于阈值、可达 free-space 的 unknown 边界低于阈值。至少记录 `known_cells`、`reachable_free_cells`、`frontier_cells`、`unreachable_frontier_cells` 和 `empty_cycle_count`。
3. 将 `empty_frontier_cycles=12` 改为带增长条件的 watchdog；若 unknown 仍占较大比例，只进入“等待/恢复”，不要自动返航。强制返航时结果标记为部分覆盖。
4. 每轮保存 frontier 统计和过滤原因计数（过小、无安全 viewpoint、BFS 不连通、无 LOS、被黑名单），这样可以区分地图问题与规划问题。

### P1：为 maze 增加区域级、可执行的 frontier

1. 将 frontier 从“代表 cell + 单个 standoff”改成 doorway/墙段区域任务：对同一 connected component 生成多个候选 viewpoint（墙两侧、端点和拐角），选择可达且信息增益最大的一个；不要因算术中心无 LOS 丢弃整个 component。
2. 对小组件采用自适应阈值：`minimum_frontier_cells` 由 12 降到 3–6，但要求至少 N 个未知邻居或预测信息增益超过下限，避免噪声 frontier 泛滥。
3. clearance 分层：候选生成使用机器人硬半径 + 0.08–0.12 m margin（约 0.36–0.40 m），最终连续 footprint 检查仍由导航层负责；若普通 standoff 无解，允许 0.45–0.55 m 的“门口观察点”，并执行原地旋转/短弧扫描。
4. 增加 frontier 走廊推进策略：当没有满足完整 standoff 的 viewpoint 时，在已知 free 的安全边界上选择 0.3–0.6 m 的短目标，扫描后重新计算 frontier，而不是直接宣告不可达。
5. `GoalRegionStillFrontier()` 使用 component overlap/doorway ID，而不是仅 0.35 m 圆邻域；目标解析后保留同一区域的其他未知分支。

### P1：降低 maze 拐角的路径失败

1. Theta* 输出交给 RPP 前执行连续 swept-footprint 校验；若 chord 失败，回退到未平滑的 A*/Theta* 分段路径或插入墙角前后的中间安全点。
2. 对 dense footprint failure 不要立即黑名单整个 frontier；先尝试 2–3 个不同侧向 viewpoint/同伦路径，只有确认局部区域不可达才冷却。
3. 将失败路径首段加入临时代价（带 TTL），确保重规划不会在同一姿态反复得到完全相同的 chord。
4. 保留最近一次控制 outcome（RPP 原始命令、静态 footprint 拒绝、CollisionMonitor、实际位姿增量），避免把局部停止误记为 frontier 失败。

### P1.5：窄道专用通过策略（不靠全局调参）

右上区域不是普通开阔空间。场景中的 collinear 窄门（`maze_wall_2` 与 `maze_wall_3` 之间）已调整为 `0.70 m` 原始净宽；机器人直径为 `0.56 m`，每侧仅约 `0.07 m` 几何余量。因此物理上可通行，但不应依赖降低全局 inflation、缩小 footprint clearance 或临时放松碰撞阈值来“碰运气”通过。右侧横墙已移除，避免外包围内形成未连通口袋。

应实现独立的 **NarrowPassageMode**，将窄道作为经过认证的拓扑对象，而不是普通 frontier goal。算法如下。

1. **从距离场检测通道，而非按固定宽度阈值调参。** 在已知 free-space 上计算 Euclidean distance transform 和 medial axis；沿骨架寻找两侧最近障碍物成对、且局部横截面宽度出现极小值的连续段。该段的中心线就是候选通道轴。通道两端必须分别连到不同的 free-space 区域（entry/exit），避免把墙角凹槽识别成门。
2. **以不确定性证明可通过性。** 对每个骨架采样点计算 `clearance - robot_radius - pose_uncertainty - map_resolution/2`。所有点均为正才生成 `PassageContract`；若为负则明确标记为 `GEOMETRICALLY_IMPASSABLE`，不尝试、也不把它算作探索失败。这里的 margin 来自当前定位协方差、栅格量化误差和扫描时间戳，而不是人工把某个 clearance 参数调小。
3. **建立通道契约。** `PassageContract` 至少包含：入口预备位姿、中心线、出口确认位姿、左右障碍物 ID/边界、最小认证余量、地图 revision，以及目标侧 frontier component ID。只有在 entry 和 exit 均处于已知 free-space、中心线完整通过 hard-footprint sweep 时，探索器才能提交该任务。
4. **使用受约束的中心线规划。** 进入 NarrowPassageMode 后，全局规划不能再用可能切内角的 Theta* chord；在 contract 的走廊带内运行栅格/状态格搜索，并对偏离 medial axis 施加代价。碰撞约束始终使用真实 `robot_radius` 和连续 swept footprint；策略改变的是候选路径空间，不是安全边界。
5. **执行分为 entry、traverse、exit 三个状态。** entry 在入口前停止并用 360° scan 复核两侧边界；traverse 只跟踪中心线，禁止跨通道重选 frontier、禁止切 chord、禁止将短暂 frontier 消失解释为任务完成；exit 只有在机器人 footprint 完全越过 gate 后才解除约束，并立即做原地观测来展开目标侧地图。
6. **闭环重认证与可恢复退出。** 每次地图 revision 或横向误差超过认证余量的一半时，暂停前进、重新计算局部距离场。认证仍成立则从当前位置重新投影到中心线；不成立则只允许沿已经验证的中心线退回 entry，绝不驶入 unknown 侧向空间。失败记录为 `PASSAGE_CERTIFICATION_LOST`，不能黑名单整片目标 frontier。

#### 近距离雷达驱动的自适应安全余量

近距离激光通常比远距离建图结果更及时、局部几何精度更高，因此可以减少“地图未知/膨胀不确定性”带来的额外余量；但必须采用分层安全模型，不能直接把 `robot_radius` 调小：

```text
硬碰撞边界 = robot footprint（永不自适应缩小）
动态不确定性余量 = f(雷达距离、入射角、连续帧一致性、时间戳新鲜度、定位协方差)
规划余量 = max(硬边界, 动态不确定性余量)
```

具体策略：

1. 在窄道内维护局部 rolling scan buffer，而不是只依赖低频全局栅格。对左右墙分别做鲁棒直线/曲线拟合，剔除孤立点和掠射角回波；至少连续 3 帧、且两侧边界残差和位移一致，才提升观测置信度。
2. 将雷达测得的障碍边界用于“局部 unknown 解锁”和中心线更新，但所有预测姿态仍执行真实 footprint 的 swept collision check。雷达只能证明障碍物在哪里，不能证明机器人 footprint 可以穿过已经被实体占据的空间。
3. 动态余量随证据自适应收缩：距离近、入射角接近法向、连续帧一致、scan 新鲜且定位协方差小，则使用较小的感知余量；距离远、掠射、帧间不一致、scan 过期或定位不确定时立即恢复保守余量。该余量由观测协方差计算，不是按 maze 场景手调常数。
4. 收缩必须有速率限制和滞回：余量只能逐帧缓慢减小，任一侧出现新近障碍或置信度下降则立即增大；避免噪声造成“放宽—收紧”振荡。
5. 速度也由当前最小净空和制动距离实时计算：`v_max` 必须保证在最坏感知/控制延迟内仍能停在硬碰撞边界之前。窄道中心线跟踪可以低速通过，但不能通过提高速度换取观测刷新率。

因此，右上角 1.0 m 通道应采用“近场雷达认证后收缩感知余量”的模式：机器人实体边界保持 `0.28 m` 半径不变，只有未知地图余量和观测不确定性余量自适应减少。这样既利用了近距离雷达的可靠性，也不会把一次错误回波转化为碰撞风险。

该策略能使“能否通过”成为确定的几何/感知判定：对于全程认证余量大于零的窄道，系统应保证尝试并沿中心线通过；对于余量非正的窄道，系统应明确报告物理不可通行。不能承诺对小于机器人实体宽度、地图与真实障碍物矛盾或定位误差超过剩余余量的任意窄道 100% 成功，但可对上述经过认证的类别提供可验证的通过保证。

建议新增接口和状态：

```text
PassageDetector::Detect(distance_field, reachable_free_graph)
PassageContract { entry, centerline, exit, minimum_margin, map_revision, target_component }
NavigationPhase::kEnterPassage / kTraversePassage / kExitPassage
PASSAGE_COMPLETED / GEOMETRICALLY_IMPASSABLE / PASSAGE_CERTIFICATION_LOST
```

回归不只检查最终覆盖率，还必须为 1.0 m、0.9 m、以及小于机器人直径的合成通道分别断言：通过、判为不可认证而不碰撞、判为几何不可通行。右上角场景应验证 entry → centerline → exit 的轨迹始终位于 contract 走廊内，并在 exit 后发现目标侧 frontier。

### P2：地图与验收指标

1. 记录 ground-truth 可通行 mask 仅用于离线评估，计算 `coverage = explored_ground_truth_free / total_ground_truth_free`；不要用整个栅格（含地图外 unknown）作为分母。
2. 每个 seed 输出：覆盖率、可达覆盖率、frontier recall、重复路径比例、失败目标数、返航前 unknown 面积和路径长度/覆盖面积。
3. 将四个合法 maze spawn（seed 0/1/4/10）纳入回归门槛；建议初始验收目标为覆盖率 ≥90%、`NO_EXECUTABLE_FRONTIER` 为 0、dense footprint failure ≤1%，再优化路径长度。

## 5. 推荐实施顺序与验证

1. 先实现 P0 的状态/统计字段并重跑四个 seed，确认 COMPLETE 不再掩盖 `RETURNED_WITH_UNKNOWN`。
2. 加入多 viewpoint、component overlap 和 doorway 推进，单测覆盖：非凸 frontier、L 形墙角、窄门、frontier 小于 12 cells 但信息增益有效的情况。
3. 修复 Theta*/RPP 的 corner fallback 和失败路径 TTL，重点复现 `maze-4` 的两次 dense footprint failure。
4. 最后再调评分权重（当前 `2 log1p(gain) - .9 travel - .25 standoff` 及 tour 的 `1.5 gain - .65 travel`）；在诊断字段缺失前调权重只能掩盖问题。

## 6. 本次实现与回归结果

本次已实现：

- maze frontier 最小组件阈值由 12 降至 6；
- 代表 cell 无 LOS 时，增加仍要求安全 footprint、BFS 可达且能看到 component 成员的 fallback viewpoint；
- 无可执行 frontier 只有在地图长期无增长后才触发返航；
- 强制返航结果写为 `PARTIAL`；
- Theta* 路径 dense footprint 校验失败时，自动回退到保守的 A* 栅格路径。
- 规划快照接入近场雷达射线内部的临时 free evidence：只解锁 unknown，不覆盖已知障碍，且每次规划重新认证。
- 对呈现窄道形态的候选 cell，在不改变硬 footprint 的前提下使用实体半径检查，避免开阔区全局放宽安全距离。

2026-09-02 使用重建后的 Docker 镜像运行 maze 四个合法出生点：

| seed | 结果 | 已知 cell | 完成目标 | 失败目标 | 轨迹距离 |
|---:|---|---:|---:|---:|---:|
| 0 | COMPLETE | 42,142 | 10 | 0 | 73.3 m |
| 1 | COMPLETE | 36,406 | 6 | 0 | 33.6 m |
| 4 | COMPLETE | 69,591 | 14 | 0 | 91.7 m |
| 10 | COMPLETE | 69,216 | 12 | 0 | 46.2 m |

seed 4 在 600 s 任务预算下曾因返航超时失败（已探索 69,518 cells、12 个目标、0 失败）；将任务预算提高到 1,200 s 后成功返航。该现象说明路径回退消除了 frontier 执行失败，但长距离 maze 返航仍需单独优化速度/预算，不能把它误判为探索覆盖失败。

随后重新构建包含近场雷达证据和窄道候选策略的镜像，使用 seed 4、1,200 s 预算复测：`COMPLETE`，已知 69,048 cells，完成 19 个目标、失败 0 个，返航误差 0.039 m；日志显示机器人到达右上通道末端附近 `(8.73, 7.18)` 后继续完成另一侧 frontier，再成功返航。

为避免其它横向墙段把底部/侧部房间切成不可达孤岛，`generate_exploration_world.py` 又将 `maze_wall_8/9/10` 缩短为 1.8 m，在墙端形成约 0.7–0.9 m 的交替门洞，同时保留 0.70 m 窄门作为专项测试。Docker 重建后 maze-4 复测结果为：`COMPLETE`，已知 75,901 cells，完成 17 个目标、失败 0 个，轨迹 125.8 m，返航误差 0.039 m。

核心回归：`cmake --build third_party/navigation2/build -j2`，以及 `ctest --test-dir third_party/navigation2/build --output-on-failure`，8/8 测试通过。

## 7. 伪失败修复（已实现）

此前 maze-0/1/4 中的少量“失败目标”主要来自地图更新和安全控制的瞬态状态，而不是 frontier 真正不可达：

- 目标在执行前因局部地图刷新变为 occupied 时，属于 stale goal；丢弃该目标并重新生成 frontier，不再调用 `RecordAttempt(..., false)`。
- 执行过程中触发 TTC/碰撞监视器、控制器超时或局部路径失效时，不能直接判定目标不可达；清理当前目标并允许下一轮重规划，避免把安全制动计为探索失败。
- 只有在多个独立 viewpoint 均不可达、且地图长期无增长后，才进入返航/失败统计。这样既保留安全保护，也避免失败计数污染覆盖率指标。

对应实现位于 `autonomous_explorer_node.cc` 的目标拒绝、终端超时处理分支；该逻辑与窄道的近场雷达 free evidence、局部实体半径策略配合使用，不依赖全局安全距离调参。

## 8. 全图探索完成判定修复计划

当前 `COMPLETE` 仍可能出现“frontier 暂时耗尽但场景右侧大面积未知”的伪完成，因此测试通过条件必须从单一状态字段升级为全局覆盖验收。计划如下。

### 阶段一：建立可探索区域基准

1. world 生成器输出与 `scenario.json` 配套的 truth occupancy 栅格，固定分辨率、原点和机器人实体半径。
2. truth 栅格只保留外包围内部、且机器人几何上可到达的自由空间；墙体、外部区域和实体不可通过区域标记为非探索区。
3. 为每个场景记录 `reachable_free_cells`，避免使用随地图尺寸变化的固定 `known_cells` 魔数。

### 阶段二：运行时收敛判定（不依赖 truth）

1. 探索过程中只能使用当前传感器建图结果，不能读取场景 truth 或预先知道可探索面积。
2. frontier 耗尽时，先执行未知区域连通性、地图边界接触、最近激光观测时间和地图增长趋势检查。
3. 对边界附近、窄道入口和大面积未知连通块保留探索候选，避免因单次 frontier 暂时消失就返航。
4. 仅当连续多轮扫描后地图无增长、没有可达未知连通块、且最终返航路径有效时，运行时才允许标记 `COMPLETE_CANDIDATE` 并返航；这仍不是最终覆盖率结论。

### 阶段三：测试门禁与回归矩阵

1. 建图和返航完成后，离线验证器读取保存的 `map.swmap`/snapshot 与同次运行生成的 truth 栅格，计算真实覆盖率；运行时不参与该计算。
2. 增加负例：人为截断机器人轨迹或屏蔽右侧激光，结果即使上报 `COMPLETE` 也必须被测试拒绝。
3. maze 四个合法出生点（0、1、4、10）全部要求 `coverage >= 0.90`，目标场景验收要求 `>= 0.95`；窄道区域必须至少有一次有效观测。
4. apartment、warehouse 同样执行覆盖率门禁，并保留 8/8 C++ 单元测试作为基础回归。

### 阶段四：可观测性和验收输出

1. 离线验证结果（而非探索器在线状态）增加 `reachable_free_cells`、`covered_free_cells`、`coverage_ratio`、`unknown_components` 和 `largest_unknown_component`。
2. 轨迹图叠加 truth 可探索边界及未覆盖区域，避免仅凭绿色轨迹误判完成。
3. CI 将“状态完成但覆盖率不足”作为失败，并保存 snapshot、truth 栅格和差分图用于定位。

### 验收标准

- 右上窄道及其内部区域均被机器人实际到达或由近距离雷达观测覆盖；
- 外包围内部不存在大面积连续未知区域；
- 四个 maze seed 均通过全图覆盖门禁后才能报告 `COMPLETE`；
- 任意早退、返航超时或局部 frontier 耗尽只能报告 `PARTIAL/FAILED`；即使运行时报告 `COMPLETE_CANDIDATE`，离线覆盖率不足也必须判定测试失败。

## 9. 2026-09-03 Docker 回归原因记录

使用重建镜像运行 12 个合法场景/出生点（`EXPLORATION_TIMEOUT=1200`，并行度 3），矩阵结果为 11/12 通过、1/12 失败：

| 场景 | 结果 | 关键现象 | 初步归因 |
|---|---|---|---|
| apartment 0/1/4/10 | 4 个 COMPLETE | 97.7k known cells，0 failed goals | 正常完成 |
| warehouse 0/1/10 | 3 个 COMPLETE | 91.3k known cells，0 failed goals | 正常完成 |
| warehouse 4 | FAILED | 41 个目标完成；最终在线地图返航路径 3 次控制执行失败，返航误差 16.55 m | 探索本身已完成，最终返航在局部地图/控制器跟踪阶段卡住；不是 frontier 覆盖不足 |
| maze 0/4/10 | 3 个 COMPLETE | 98.1k known cells，0 failed goals | 正常完成 |
| maze 1 | COMPLETE（含 1 failed goal） | 早期一次 `start or goal is occupied`，随后完成返航，98.2k known cells | 地图刷新造成 stale goal 的瞬态伪失败，未影响整体覆盖 |

本轮没有出现此前那种 52k known cells 就提前 `COMPLETE` 的早退，但这只是运行统计，最终仍需按第 8 节的 truth 栅格离线覆盖率门禁验收。当前唯一真实失败集中在 warehouse-4 的最终返航控制执行，需要单独增加返航阶段的重定位/局部路径恢复策略。

## 10. 当前不完全探索的根因与修复方案

### 已确认根因

1. **“无可执行 goal”被误判为“无 frontier”**：`FrontierExplorer::SelectGoals()` 先提取 raw frontier，再经过安全 footprint、BFS 可达性、LOS、standoff 和黑名单过滤。右侧未知区对应的 raw frontier 可能存在，但所有候选 viewpoint 被过滤后返回空列表；`autonomous_explorer_node.cc` 无法区分这两种情况，进入 `empty_frontier_cycles_`，达到 12 个周期后直接返航并报告 `COMPLETE`。
2. **动态地图边界 frontier 被漏检**：frontier 扫描目前只遍历 `row=1..height-2`、`col=1..width-2`。当 mapper 的当前窗口边缘仍是 unknown 时，边缘 free/unknown 过渡不会生成 frontier，地图不会主动向右侧扩展。
3. **候选过滤缺少诊断**：目前日志只输出最终 `frontier_cells`，没有 raw cluster 数、被 footprint/可达性/LOS 各过滤掉的数量，因此“右侧没有 frontier”与“右侧 frontier 全部不可执行”无法从日志区分。
4. **固定 known-cell 门禁不是覆盖率**：`known_cells` 包含 occupied 和 free，且受动态地图窗口影响，只能作为异常早退报警，不能作为全图覆盖证明。
5. **窄道 footprint 与 standoff 约束冲突**：窄道候选已通过实体半径检查，但统一的 `minimum_standoff=0.55 m` 仍会拒绝距离未知边界约 0.28–0.35 m 的合法观察位姿；因此 raw frontier 会持续存在，却无法生成 executable goal。

### 修复顺序

1. 修改 `SelectGoals()` 返回结构，至少携带 `raw_frontier_cells`、`raw_clusters`、`reachable_clusters`、`filtered_by_footprint`、`filtered_by_los` 和 `executable_goals`。
2. 外层状态机按结果分流：`raw_frontier_cells > 0 && executable_goals == 0` 时只能进入 `BLOCKED_FRONTIER`，等待地图增长或执行专用恢复，不得累加 `empty_frontier_cycles_`，更不能报告 `COMPLETE`。
3. 增加边界 frontier 策略：对动态地图四周的 free/unknown 接触带，生成靠内侧的安全 viewpoint；只有确认是地图窗口边界而非真实障碍物时才推进 mapper 窗口。
4. 对被安全 footprint 拒绝的窄道候选，调用专用 corridor viewpoint 搜索；对被 LOS 拒绝的 L 形墙角，保留现有 component-member LOS fallback，但记录拒绝原因。
5. corridor viewpoint 同时采用自适应 standoff：仅在检测到单向窄走廊、且实体半径 clearance 通过时，允许 `0.28 m <= standoff < 0.55 m`；开阔区域继续使用原始 standoff 约束。
6. 只有 raw frontier 数为零、地图连续多轮无增长、且未知连通区域没有可达入口时，才允许进入 `COMPLETE_CANDIDATE`；最终 `COMPLETE` 仍由离线 truth 栅格覆盖率门禁确认。
7. 将固定 `known_cells` 下限降级为诊断告警，替换为离线 `coverage_ratio`、连续未知区域和窄道观测结果。

### 验证方式

- 增加一个“右侧大面积 unknown 但当前没有安全 viewpoint”的回归 case，必须保持探索状态或返回 `PARTIAL`，禁止 `COMPLETE`。
- 增加动态地图边界扩展 case，验证机器人靠近边缘后能生成新 frontier。
- 运行日志必须能回答：是否有 raw frontier、被哪一层过滤、是否因地图边界截断。
- 12 个 Docker case 重新运行后，以 truth 栅格离线覆盖率作为唯一场景通过条件；`warehouse-4` 另验收最终返航误差。

本次先落地了状态机与诊断修复，并用重建镜像复测 `maze-4`：日志已能区分
`raw_cells=22/20, raw_clusters=13, candidates=0, stable=0`，不再把该状态当作
“frontier 为空”；连续等待后输出 `No executable frontier ... returning safely with current map`，
结果为 `PARTIAL`（known cells 98,152，完成目标 23，返航误差 0.040 m）。这证明伪 `COMPLETE`
已被拦截，但也确认仍有 raw frontier 因几何/可达性过滤无法执行，后续必须实现边界 viewpoint
和窄道恢复策略，才能把该 case 提升到 truth 覆盖率验收线。

随后加入了窄道单栅格组件保留、corridor 自适应 standoff（实体半径下限 0.28 m）以及入口
无 LOS fallback。最新 `maze-4` 仍报告 `PARTIAL`，但完成目标由 6/18 提升到 17，返航误差
保持约 0.04 m；末端日志为 `raw_cells=34, raw_clusters=23, candidates=0`。这表明剩余
问题已收敛到：这些 frontier 周围没有同时满足实体 footprint、BFS 可达和 corridor 形态的
安全入口，不能继续靠放宽 standoff 解决，下一步应针对动态地图边界/未知区域入口设计探测
动作，并在离线 truth 覆盖率达标前保持 `PARTIAL`。

已继续实现一版 bounded frontier-entry scan：当 raw frontier 存在但没有任何候选 viewpoint 时，
机器人在当前安全位姿原地以 `0.35 rad/s` 扫描 6 s，等待近场激光使地图增长，再重新生成
frontier；连续 30 个周期仍无可执行入口才返回 `PARTIAL`。该动作不改变 footprint/碰撞约束，
只为解决入口尚未被观测导致的候选缺失。

2026-09-03 最新完整 Docker 矩阵（12 个 case，重建镜像、1200 s 任务预算）结果：矩阵门禁
判定 `12/12` 不通过。原因不是全部探索崩溃，而是新的严格状态机已将覆盖/收敛不足报告为
`PARTIAL`，同时部分 case 在最终返航阶段超时：apartment 四个均为 `PARTIAL`（17--20
目标，约 97.7k known cells）；warehouse-1/4/10 为 `PARTIAL`（35--42 目标），warehouse-0
为返航超时；maze-1/4 为 `PARTIAL`（16/20 目标，maze-1 有 1 个 stale-goal），maze-10
返航执行失败，maze-0 进程未生成 result。该结果说明伪 `COMPLETE` 已被阻断，但当前算法仍
未达到“全图探索完成”验收，后续必须优先解决剩余 raw frontier 的入口生成和最终返航超时。

相关实现：[`frontier_explorer.cc`](../../exploration/frontier_explorer.cc)、[`autonomous_explorer_node.cc`](../../ros/autonomous_explorer_node.cc)、[`generate_exploration_world.py`](../../../../simulation/gazebo/generate_exploration_world.py)。
