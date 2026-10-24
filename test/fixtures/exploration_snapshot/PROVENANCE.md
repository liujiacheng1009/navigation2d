# Frozen planning snapshots

Generated from navigation2d commit a947deba, original
`ros/autonomous_explorer_node.cc::StartNavigation`, before refactoring.
Eight cases cover rotated pose, shifted map origin, doorway mode, trail
painting and return. The probe stops after writing the original map snapshot,
before navigator construction or any command computation. ROS domain 211 was
used without spinning or interacting with the running simulator.

Input generation is reproduced by `exploration_snapshot_replay_test.cc`.
Keep these frozen expected cells; do not regenerate from the refactored builder.
Full frozen source archive and binaries are held in the local ignored
`artifacts/exploration-refactoring/baseline/` of the parent repository.
