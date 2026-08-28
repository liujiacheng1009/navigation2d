# 全局规划器升级路线图

目标是为 2D 差速移动机器人提供确定性、运动学可行、代价感知、可增量修复且具有可靠降级路径的
全局规划栈。默认目标架构为 SE(2) State Lattice；cost-aware 2D A* 保留为圆形机器人和低算力
平台的稳定后备。短时动态障碍仍由局部 MPC 负责，全局层只响应持续封路和拓扑变化。

## 实施阶段和验收条件

1. **搜索基线修正（已完成）**：footprint clearance 必须参与几何碰撞检查；Theta* 捷径对整段
   costmap 代价积分；输出路径中间点使用路径切线 yaw，同时保留目标 yaw。
2. **通用搜索内核（已完成）**：搜索队列、节点存储、邻接扩展和路径恢复解耦；同一内核可承载 2D、SE(2)
   和增量节点；提供确定性 tie-break、节点/时间预算及诊断指标。
3. **Distance field 与 footprint lookup（已完成）**：静态和动态致命栅格生成欧氏距离场；圆形 footprint
   O(1) 查询；多边形 footprint 按 yaw bin 预栅格化并支持 swept-volume 查询。
4. **差速 State Lattice（已完成）**：生成直行、圆弧、原地旋转及可选倒车原语；离散终点闭合；每条原语
   保存长度、曲率、方向切换和 footprint 采样。
5. **Anytime 搜索（已完成）**：加入障碍启发式反向 DP 缓存、Weighted A* 首解和 ARA* 逐步改善；报告
   次优界、展开数、首解时间和最终时间。
6. **约束平滑器（已完成）**：联合优化长度、二阶平滑度、曲率变化和障碍距离；不得越出原路径安全走廊，
   保留起终点、目标姿态、倒车 cusp 和原地旋转段。
7. **增量修复（已完成）**：以目标反向的 D* Lite（LPA* 的增量重规划形式）复用相同目标下的
   `g/rhs/OPEN/km`，只更新 costmap 变化影响的运动原语源状态；地图
   大范围变化或缓存不一致时完整重建。
8. **统一基准（已完成）**：覆盖大地图、窄通道、死胡同、动态封路和圆形/多边形 footprint；比较成功率、
   路径长度、最小净空、曲率、首解/最终 P50/P95/P99、展开节点、重规划复用率和峰值内存。

## 发布门槛

- 所有返回路径逐段 footprint 碰撞检查通过，且终点姿态保持不变。
- 固定输入、配置和 costmap digest 下结果确定。
- 超时必须返回已有安全路径或明确失败，不得返回半构造路径。
- State Lattice 失败或资源超限时自动降级到 cost-aware 2D A*。
- 新规划器进入默认配置前，统一基准成功率不得低于现有 A*，P95 规划时间必须满足产品预算。

首轮发布矩阵中 State Lattice 为 5/5、A* 为 2/5、Theta* 为 3/5；因此 State Lattice 已设为
默认。大地图单次样本峰值约 177 MiB、重规划 P95 约 296 ms，低算力产品仍应按自身预算复测，
必要时显式选择 `astar`。

## 规划诊断

benchmark JSON 输出 `global_plan_expansions`、`global_plan_generated`、
`global_plan_first_solution_seconds`、`global_plan_seconds`、
`global_plan_suboptimality_bound` 和 `obstacle_heuristic_cache_hits`，用于验证 anytime 首解、最终
改善质量和重复重规划缓存收益。`global_plan_incremental_reuse`、
`global_plan_repaired_states`、`global_plan_incremental_replans` 和
`global_plan_repaired_states_total` 用于验证移动起点与地图变更时的增量修复范围。
`global_plan_p50_us/P95/P99`、`global_plan_first_solution_p50_us/P95/P99`、
`global_plan_expansions_total`、`path_min_clearance_m`、`path_max_curvature`、`peak_rss_kb` 和
`global_plan_fallback_used` 构成发布基准指标。净空是按外接圆计算的保守值。

统一比较命令为：

```bash
python3 tools/run_global_planner_benchmark.py --repetitions 3 --jobs 5
```

每个进程有 120 秒墙钟上限；超限作为该规划器在该场景的失败样本进入成功率，而不会挂死整套基准。
地图 JSON 可用逐格 `cells`，也可用紧凑的 `rectangles: [[x0,y0,x1,y1,value], ...]` 描述
`[x0,x1) × [y0,y1)` 障碍矩形。
