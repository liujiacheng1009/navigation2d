# Upstream control-core provenance

The ROS-free controller adapters in this directory track these upstream projects:

- Nav2 MPPI controller, commit `a143bcf9860273421f1918e525fc617af947c009`, MIT.
- TUD-AMR Guidance Planner, commit `2c4188371e18e2fb3d083e0867b5e4d537a42860`, Apache-2.0.
- DecompUtil, commit `b0836c7228d19f0fa97282c584b55adf642279da`, BSD-3-Clause.

Nav2's public controller is coupled to ROS messages, pluginlib and Nav2's costmap. The numerical
MPPI algorithm, control-sequence shifting, differential-drive model, Savitzky-Golay filtering and
critic semantics are adapted behind Navigation2D's ROS-free interfaces. Modified files retain the
upstream copyright notice and identify the adaptation.

Guidance Planner and DecompUtil integration likewise keeps ROS transport outside the product core.
Do not update an upstream commit without rerunning controller unit tests and the complete control
benchmark matrix.
