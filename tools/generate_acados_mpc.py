#!/usr/bin/env python3
"""Generate the C++ acados RTI solver for Navigation2D's unicycle MPCC.

Requires ACADOS_SOURCE_DIR and its acados_template package, plus CasADi.  The
generated directory is intentionally not committed; it is a build artifact
selected by CMake when NAVIGATION2D_ACADOS_GENERATED_DIR is supplied.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver


def build_solver(output: Path, horizon: int, duration: float, obstacle_slots: int,
                 corridor_slots: int) -> None:
    model = AcadosModel()
    model.name = "navigation2d_mpcc"
    # [x, y, yaw, v, w], u = [linear_acceleration, angular_acceleration].
    state = ca.SX.sym("state", 5)
    control = ca.SX.sym("control", 2)
    parameters = ca.SX.sym("parameters", 4 + 5 * obstacle_slots + 4 * corridor_slots)
    x, y, yaw, velocity, angular = state[0], state[1], state[2], state[3], state[4]
    acceleration, angular_acceleration = control[0], control[1]
    model.x, model.u, model.p = state, control, parameters
    model.f_expl_expr = ca.vertcat(velocity * ca.cos(yaw), velocity * ca.sin(yaw), angular,
                                   acceleration, angular_acceleration)

    # p = reference [x, y, yaw, v], followed by fixed-capacity obstacle slots
    # [x, y, rx, ry, active]. rx/ry include robot/obstacle radius, confidence
    # inflation and safety margin. Fixed capacity keeps generated dimensions
    # deterministic while supporting dense tracker output.
    ref_x, ref_y, ref_yaw, ref_v = parameters[0], parameters[1], parameters[2], parameters[3]
    heading_error = ca.atan2(ca.sin(yaw - ref_yaw), ca.cos(yaw - ref_yaw))
    model.cost_y_expr = ca.vertcat(x - ref_x, y - ref_y, heading_error, velocity - ref_v,
                                   angular, acceleration, angular_acceleration)
    model.cost_y_expr_e = ca.vertcat(x - ref_x, y - ref_y, heading_error, velocity - ref_v, angular)
    # Safe set is h >= 1. A disabled obstacle has a deliberately distant centre.
    obstacle_constraints = []
    for slot in range(obstacle_slots):
        offset = 4 + 5 * slot
        obstacle_x, obstacle_y = parameters[offset], parameters[offset + 1]
        obstacle_rx, obstacle_ry = parameters[offset + 2], parameters[offset + 3]
        obstacle_active = parameters[offset + 4]
        obstacle_constraints.append(
            ((x - obstacle_x) / obstacle_rx) ** 2 + ((y - obstacle_y) / obstacle_ry) ** 2
            + (1. - obstacle_active) * 1e6
        )
    corridor_constraints = []
    corridor_offset = 4 + 5 * obstacle_slots
    for slot in range(corridor_slots):
        offset = corridor_offset + 4 * slot
        normal_x, normal_y = parameters[offset], parameters[offset + 1]
        bound, active = parameters[offset + 2], parameters[offset + 3]
        corridor_constraints.append(
            bound - normal_x * x - normal_y * y + (1. - active) * 1e6
        )
    model.con_h_expr = ca.vertcat(*(obstacle_constraints + corridor_constraints))

    ocp = AcadosOcp()
    ocp.name = "navigation2d_mpcc"
    ocp.model = model
    ocp.parameter_values = np.zeros(4 + 5 * obstacle_slots + 4 * corridor_slots)
    ocp.solver_options.N_horizon = horizon
    ocp.solver_options.tf = duration
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.cost.W = np.diag([12.0, 12.0, 3.0, 1.5, 0.2, 0.08, 0.04])
    ocp.cost.W_e = np.diag([24.0, 24.0, 6.0, 3.0, 0.4])
    ocp.cost.yref = np.zeros(7)
    ocp.cost.yref_e = np.zeros(5)
    ocp.constraints.x0 = np.zeros(5)
    ocp.constraints.idxbu = np.array([0, 1])
    ocp.constraints.lbu = np.array([-0.8, -1.8])
    ocp.constraints.ubu = np.array([0.8, 1.8])
    ocp.constraints.idxbx = np.array([3, 4])
    ocp.constraints.lbx = np.array([-0.12, -0.9])
    ocp.constraints.ubx = np.array([0.55, 0.9])
    ocp.constraints.lh = np.concatenate((np.ones(obstacle_slots), np.zeros(corridor_slots)))
    ocp.constraints.uh = np.full(obstacle_slots + corridor_slots, 1e15)
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.print_level = 0
    ocp.code_gen_options.code_export_directory = str(output)
    ocp.code_gen_options.json_file = str(output / "acados_ocp.json")
    # Generate/build separately: this emits a C/C++ consumable solver and
    # does not require Python to dlopen it during generation.
    AcadosOcpSolver.generate(ocp, verbose=False)
    AcadosOcpSolver.build(str(output), verbose=False)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("generated/acados_navigation2d_mpcc"))
    parser.add_argument("--horizon", type=int, default=20)
    parser.add_argument("--duration", type=float, default=1.2)
    parser.add_argument("--obstacle-slots", type=int, default=4)
    parser.add_argument("--corridor-slots", type=int, default=8)
    args = parser.parse_args()
    if (args.horizon < 2 or args.duration <= 0 or args.obstacle_slots < 1 or
            args.corridor_slots < 4):
        parser.error("horizon >= 2, positive duration, obstacle-slots >= 1 and corridor-slots >= 4 required")
    if not os.environ.get("ACADOS_SOURCE_DIR"):
        parser.error("ACADOS_SOURCE_DIR must point to an acados source tree")
    args.output.mkdir(parents=True, exist_ok=True)
    build_solver(args.output.resolve(), args.horizon, args.duration, args.obstacle_slots,
                 args.corridor_slots)


if __name__ == "__main__":
    main()
