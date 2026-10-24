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
