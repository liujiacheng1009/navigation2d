# A dependency edge describes the actual consumer, rather than pulling every
# implementation and SDK into each domain consumer through navigation2d_core.
function(navigation2d_library name)
  add_library(${name} STATIC ${ARGN})
  target_link_libraries(${name} PUBLIC navigation2d_values)
  target_compile_options(${name} PRIVATE -Wall -Wextra -Wpedantic -Werror)
endfunction()

navigation2d_library(navigation2d_costmap costmap/grid_2d.cc costmap/layered_costmap.cc
  costmap/obstacle_layer.cc costmap/grid_change_history.cc)
target_link_libraries(navigation2d_costmap PUBLIC navigation2d_geometry)

navigation2d_library(navigation2d_planning
  planning/search_core.cc planning/incremental_search.cc planning/collision_checker.cc
  planning/obstacle_heuristic.cc planning/grid_search.cc planning/motion_primitives.cc
  planning/path_smoother.cc planning/dijkstra_planner.cc planning/astar_planner.cc
  planning/state_lattice_planner.cc planning/theta_star_planner.cc
  planning/path_geometry.cc planning/path_resampling.cc planning/route_identity.cc planning/path_metrics.cc)
target_link_libraries(navigation2d_planning PUBLIC navigation2d_costmap)

navigation2d_library(navigation2d_control
  control/regulated_pure_pursuit.cc control/pursuit_alignment.cc control/pursuit_collision.cc
  control/pursuit_parameters.cc control/pursuit_path_progress.cc control/pursuit_regulation.cc
  control/dwa_controller.cc control/nav2_mppi_core.cc
  control/mppi_controller.cc control/mppi_parameters.cc control/dynamic_safety.cc control/safe_corridor.cc
  control/collision_monitor.cc control/collision_monitor_parameters.cc control/range_observation.cc control/collision_monitor_geometry.cc
  control/depth_observation.cc control/mpc_controller.cc control/mpc_parameters.cc)
target_link_libraries(navigation2d_control PUBLIC navigation2d_planning)
target_include_directories(navigation2d_control SYSTEM PRIVATE "${decomp_util_SOURCE_DIR}/include")
include("${CMAKE_CURRENT_LIST_DIR}/mpc_backend.cmake")
navigation2d_configure_mpc_backend(navigation2d_control)

navigation2d_library(navigation2d_exploration
  exploration/exploration_policy.cc exploration/frontier_explorer.cc exploration/live_route_safety.cc
  exploration/frontier_tracks.cc exploration/frontier_visit_memory.cc exploration/region_scheduler.cc exploration/region_identity.cc exploration/region_partition.cc exploration/region_graph_observation.cc
  exploration/frontier_travel_graph.cc exploration/bootstrap_frontier_travel.cc
  exploration/frontier_reachability.cc exploration/viewpoint_settlement.cc
  exploration/entry_probe_search.cc exploration/entry_probe_direction.cc
  exploration/entry_probe_grid.cc exploration/mouth_gap.cc
  exploration/free_space_distances.cc exploration/unvisited_space.cc
  exploration/mouth_goal_grouping.cc exploration/mouth_goal_placement.cc
  exploration/leftover_viewpoints.cc exploration/leftover_candidate.cc
  exploration/frontier_components.cc exploration/frontier_boundary_queries.cc
  exploration/frontier_candidate_stage.cc exploration/frontier_debug_snapshot.cc
  exploration/frontier_approach_viewpoints.cc
  exploration/frontier_viewpoint_search.cc
  exploration/frontier_visibility.cc
  exploration/frontier_visible_viewpoint.cc
  exploration/planning_grid_view.cc)
target_link_libraries(navigation2d_exploration PUBLIC navigation2d_geometry)

# Generic registry knows interfaces and metadata, without concrete backends.
navigation2d_library(navigation2d_algorithm_contracts planning/planner_contract.cc control/controller_contract.cc)
target_link_libraries(navigation2d_algorithm_contracts PUBLIC navigation2d_costmap)
navigation2d_library(navigation2d_algorithm_registry application/algorithm_registry.cc)
target_link_libraries(navigation2d_algorithm_registry PUBLIC navigation2d_algorithm_contracts)
target_link_libraries(navigation2d_planning PUBLIC navigation2d_algorithm_contracts)
target_link_libraries(navigation2d_control PUBLIC navigation2d_algorithm_contracts)
navigation2d_library(navigation2d_builtin_algorithms
  application/builtin_algorithms.cc planning/planner_factory.cc)
target_link_libraries(navigation2d_builtin_algorithms PUBLIC
  navigation2d_algorithm_registry navigation2d_control navigation2d_planning navigation2d_configuration)

navigation2d_library(navigation2d_configuration
  application/navigation_config.cc application/navigation_configuration.cc
  configuration/navigation_yaml.cc configuration/planning_yaml.cc configuration/control_yaml.cc
  configuration/safety_yaml.cc configuration/navigation_validation.cc configuration/vehicle_config.cc
  configuration/grid_file.cc configuration/builtin_parameters.cc)
target_link_libraries(navigation2d_configuration PUBLIC
  navigation2d_algorithm_registry navigation2d_costmap PRIVATE yaml-cpp)

# Compatibility product facade: callers keep linking navigation2d_core. Domain
# targets above can be built and linked without this aggregate entry point.
navigation2d_library(navigation2d_core
  application/execution_watchdog.cc application/navigation_failure.cc application/navigation_failures.cc application/navigation_algorithms.cc
  application/command_safety_pipeline.cc application/docking_servo.cc
  application/navigation_motion.cc application/alignment_execution.cc
  application/docking_execution.cc application/tracking_execution.cc
  application/command_stall_watch.cc application/route_preparation.cc
  application/navigation_system.cc)
target_link_libraries(navigation2d_core PUBLIC navigation2d_builtin_algorithms
  navigation2d_configuration navigation2d_exploration)
