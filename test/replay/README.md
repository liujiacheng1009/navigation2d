# Controlled explorer replay

This test drives the actual old/new `Tick()` implementations with a fixed clock,
map, lidar and differential-drive fixture. It is not a ROS bag replay or a test of
DDS scheduling, full mission completion, or physical chassis collision clearance.
The normal case runs 600 cycles; the stuck case suppresses motion at cycles
180–259 and can finish earlier. A matching failure is equivalence evidence, not
mission success.

## Build

Run in the same sourced ROS environment as the parent `online_mapping` build.
First build the current `sweepnav_autonomous_explorer` target. Independently build
the frozen baseline `navigation2d_core` from the original revision; do not reuse
the candidate core. The original baseline is
`a947deba48765ee4dc57168ebcf1be5e2c49e205`.

```bash
python3 /opt/sweepnav/navigation2d/tools/build_explorer_replays.py \
  --build /workspace/artifacts/desktop-build \
  --baseline-core /tmp/exploration-baseline-build/libnavigation2d_core.a \
  --output /tmp/navigation-refactor-replay
```

The builder uses the real target's compiler/linker options, replaces the baseline
core link input, and records core/source/binary SHA256 digests. The baseline archive
must actually correspond to `--baseline-ref`; the caller owns that provenance.
Baseline source must be available in the local Git history, or supplied with
`--baseline-source /path/to/frozen/ros/autonomous_explorer_node.cc` when the
container bind mount does not include usable Git metadata. Changing the baseline
revision also requires a separately built matching core.

Only temporary test sources expose node internals, replace the clock and redirect
publisher calls into `ReplayOutputs`. No production file or access control is
changed. Unrecognized publisher calls abort generation. Node constructor inputs
and all execution decisions remain the actual implementations. ROS publishers are
still constructed, but no captured command is published and no executor is spun.
Run on a separate ROS domain; no simulator or vehicle is started by these tools.

## Compare behavior and timing

Create an explicit argument file (use absolute input paths):

```json
[
  ["normal", "--ros-args", "--log-level", "fatal", "-p", "vehicle_config:=/opt/sweepnav/navigation2d/config/navigation2d.yaml"],
  ["stuck", "--ros-args", "--log-level", "fatal", "-p", "vehicle_config:=/opt/sweepnav/navigation2d/config/navigation2d.yaml"]
]
```

Save it as `/tmp/explorer-replay-cases.json`. Set `ROS_DOMAIN_ID=211` and
`SWEEPNAV_EXPLORATION_RESULT` to a temporary result path. For isolated failure
result comparison, run each executable once with separate result paths and compare
the JSON files; the benchmark comparator compares the executable stdout.

```bash
export ROS_DOMAIN_ID=211
export SWEEPNAV_EXPLORATION_RESULT=/tmp/explorer-replay-result.json
taskset -c 13 python3 /opt/sweepnav/navigation2d/tools/compare_benchmarks.py \
  --baseline /tmp/navigation-refactor-replay/baseline \
  --cases /tmp/explorer-replay-cases.json \
  --metrics tick_p50_us tick_p95_us tick_p99_us wall_seconds peak_rss_kb \
  --scope controlled-explorer-Tick \
  --input-files /opt/sweepnav/navigation2d/config/navigation2d.yaml \
  --calibrate --output /tmp/explorer-aa.json
```

After A/A finishes, repeat with the same arguments and replace `--calibrate` with
`--candidate /tmp/navigation-refactor-replay/candidate --calibration /tmp/explorer-aa.json`;
use a different `--output` file for A/B. Select an available CPU and keep it fixed
across both runs. Stop builds before measurement. At least 30 pairs are required;
metric names, scope, input files, baseline binary and cases are frozen by A/A.
Do not overwrite the calibration report with the comparison report.

The trace preserves pose, commands, phase, known cells, goal outcomes, publication
counts, entry/return budgets, and a cumulative hash of status/frontier strings and
debug grids. Only the measured `controller_solve_us` and `planner_elapsed_s` fields
inside status JSON are normalized. Other ROS message types currently contribute
publication counts only. This is a deliberate limit, not full message equivalence.
Per-cycle samples and process peak RSS are reported; wall time covers ROS setup,
fixture execution and reporting up to the metric, excluding shutdown and spawn.

The scenario headers separate input-world state from output capture and execution
order. When production state ownership changes, update the explicit adapter and
compare against frozen traces before using the new fixture as a baseline. Never
change a fixture to conceal a production behavior difference.

## Message coverage

The historical per-cycle trace retains the original status/frontier/grid hash.
`path_messages_hash` additionally fingerprints every Path, PoseStamped and
PoseArray publication in order, including headers, simulated timestamps and all
pose fields as exact floating-point bits. This checks the path-message adapter
without changing the historical trace columns. The comparator treats this field
as behavior, never as timing. Regenerate both executables and A/A calibration
when the fixture changes; the builder rejects fixture edits during compilation.
Other message types still contribute only publication counts. This fingerprint
is a regression check, not evidence of DDS delivery or full mission success.

The parent ROS build also provides `exploration_ros_paths_test`; run that binary
to verify offsets, window fallback, cursor clearing, tour duplicate handling and
live-map wall thresholds. Core-only builds do not require ROS for these adapters.

Pose observations enter the candidate through `PoseFeedback::ObserveLocalization`.
The frozen node uses the equivalent historical accumulation in a generated test
hook. The fixture cannot mutate the candidate's private pose/trail storage. The
parent target `exploration_pose_feedback_test` separately covers the rotated
truth frame, header preservation, strict 5 cm sampling and accumulated distance.

## Compile cache

The builder preprocesses each generated translation unit and checks its complete
input digest, codegen flags, compiler/frontend/assembler identities and object
integrity before reusing compilation. New include shadows invalidate the cache.
Linking always runs with the current libraries. The original source path and
compiler flags are retained; the frozen baseline binary digest stayed identical
when enabling the cache. `--no-object-cache` forces a fresh object build. Cache
inputs and hit/miss information are recorded alongside the replay manifest.

### 入口流程对照

生成的 `baseline entry` 与 `candidate entry` 使用真实探索器、导航会话和固定地图，直接调用入口流程，比较靠近、无探测点、启动探测、已入室与冷却场景的结果、预算、导航状态和发布摘要。该场景补足 normal/stuck 未进入入口分支的缺口；不替代实际穿门的 ROS 闭环。运行时沿用相同 ROS 参数，逐项比较 JSON，不能只检查进程退出码。

```bash
ROS_DOMAIN_ID=211 python3 tools/check_entry_replay.py \
  --baseline /tmp/navigation-refactor-replay/baseline \
  --candidate /tmp/navigation-refactor-replay/candidate \
  --output /tmp/entry-parity.json -- \
  --ros-args --log-level fatal -p vehicle_config:=config/navigation2d.yaml
```

`entry-benchmark` 执行同样五种分支，分别输出 `approach_us`、`unavailable_us`、`probe_us`、`inside_us`、`cooling_us`，并保留完整行为 trace、进程 wall time 和峰值 RSS。计时包围实际入口调用，未将构造节点算进各分支耗时；进程指标仍包含构造。使用 compare_benchmarks 的显式指标/范围、冻结 A/A 后交替 A/B，不将此场景视为穿门运动闭环。

### 复用冻结基线包

将已生成并表征过的 `baseline`、`baseline.cc`、`manifest.json` 保存在独立目录；后续构建可传 `--baseline-bundle <目录>`。工具逐项核对原始源码、独立核心库、场景、生成源码和二进制摘要，复用该基线，只重建候选。基线包与输出目录必须互不包含，原文件不被覆盖。这样候选新增的 ROS 适配器或后续私有布局变化不会被重新链接/编译进基线；基线包仍需来自真实的独立冻结构建，不能用候选程序冒充。

### 无端点逻辑测量（独立范围）

默认回放保留端点构造，仅捕获发布；ROS/DDS 后台工作仍可能混入 wall time。
`--logic-only` 额外移除测试程序中未被执行器消费的发布器、订阅和定时器构造，
关闭 rosout、参数服务和参数事件发布，保留参数校验及初始 WAITING 事件。
生产节点不变。该模式必须用独立 A/A、范围名称及基线包，不能沿用默认模式的
校准、覆盖其不确定报告或宣称真实 ROS 调度已经通过。先比较两种模式完整的
normal/stuck/entry 行为，确认测试启动方式没有改变输入/输出，再测量逻辑成本。

新编基线还必须使用原版本的完整头文件树：可传 `--baseline-include-root`，或由
完整冻结源码路径 `.../navigation2d/ros/autonomous_explorer_node.cc` 推断其父根目录。
生成工具记录头文件摘要并检查构建期间未改变，基线链接剔除候选侧 ROS 适配对象。
只有一份旧 `.cc` 而没有配套头文件/核心库，不能作为新的独立基线构建。
已有经验证的基线包继续按其二进制与来源摘要复用；两种执行模式不能混用基线包。
