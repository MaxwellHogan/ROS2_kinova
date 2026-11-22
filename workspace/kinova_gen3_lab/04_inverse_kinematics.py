"""EXERCISE 4: position-only inverse kinematics with damped least squares.

Solves offline using the URDF and *only then* sends a short joint trajectory.
Optional: python3 04_inverse_kinematics.py --x 0.2 --y 0.1 --z 0.4
Default target is constructed from a nearby joint configuration and is known
reachable in the robot's model. Always keep simulation clear of obstacles.
"""
import argparse
import numpy as np
import rclpy
from kinova_model import KinovaModel
from kinova_ros import KinovaROS


def numerical_jacobian(model, q, epsilon=1e-4):
    p = model.position(q)
    J = np.zeros((3, 6))
    for col in range(6):
        shifted = q.copy()
        # TODO 1: perturb joint `col` by epsilon
        # shifted[col] += ...

        # TODO 2: finite difference: J[:, col] = (FK(shifted) - p) / epsilon
        pass
    return J


def solve_ik(model, initial_q, xyz_target, max_iterations=150):
    q = initial_q.copy()
    damping = 0.07
    for iteration in range(max_iterations):
        # TODO 3: calculate current XYZ and target error
        error = None
        if error is None:
            raise NotImplementedError("Complete the target error TODO")
        if np.linalg.norm(error) < 0.005:
            return q, True, iteration
        J = numerical_jacobian(model, q)

        # TODO 4: DLS update:
        # delta_q = J.T @ np.linalg.solve(
        #      J @ J.T + (damping ** 2) * np.eye(3), error)
        delta_q = None
        if delta_q is None:
            raise NotImplementedError("Complete the damped least squares TODO")

        # Limit each numerical iteration to <= 0.04 rad and respect URDF limits
        q += np.clip(delta_q, -0.04, 0.04)
        q = np.clip(q, model.lower, model.upper)
    return q, False, max_iterations


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float)
    parser.add_argument("--y", type=float)
    parser.add_argument("--z", type=float)
    parser.add_argument("--execute", action="store_true", help="Actually command the simulator")
    args = parser.parse_args()
    if any(v is not None for v in (args.x, args.y, args.z)) and not all(
        v is not None for v in (args.x, args.y, args.z)
    ):
        parser.error("Provide all of --x, --y and --z, or none")

    rclpy.init()
    arm = KinovaROS()
    try:
        initial_q = arm.wait_for_state()
        model = KinovaModel()
        if args.x is None:
            # Safe *mathematical* nearby target; still inspect it before execution
            q_example = initial_q.copy()
            q_example[0] += 0.12
            q_example[1] += 0.10
            if not model.valid_limits(q_example):
                raise ValueError("Example joint target exceeds URDF limits")
            target = model.position(q_example)
        else:
            target = np.array([args.x, args.y, args.z])

        result_q, converged, iterations = solve_ik(model, initial_q, target)
        error = np.linalg.norm(model.position(result_q) - target)
        max_change = np.max(np.abs(result_q - initial_q))
        print("Starting XYZ [m]:", np.round(model.position(initial_q), 4))
        print("Target XYZ [m]  :", np.round(target, 4))
        print("Solved XYZ [m]  :", np.round(model.position(result_q), 4))
        print("Error [m]       :", round(float(error), 5))
        print("Iterations      :", iterations)
        print("Converged       :", converged)
        print("Joint change [rad]:", np.round(result_q - initial_q, 4))

        if not args.execute:
            print("DRY RUN: add --execute only after inspecting the result")
            return
        if not converged or error >= 0.01 or max_change > 0.35 or not model.valid_limits(result_q):
            raise ValueError("Refusing trajectory: poor convergence, joint change or limit")
        arm.send(result_q, duration=5)
        reached = arm.wait_for_target(result_q, timeout=12)
        if arm.q is not None:
            actual_error = np.linalg.norm(model.position(arm.q) - target)
            print("Final measured XYZ [m]:", np.round(model.position(arm.q), 4))
            print("Measured XYZ error [m]:", round(float(actual_error), 5))
        print("Joint trajectory reached:", reached)
    finally:
        arm.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
