"""EXERCISE 5 / optional: use SciPy to solve XYZ inverse kinematics.

This adapts the optimisation direction of the supplied 7-DOF ROS 1 scripts to
six Kinova Gen3 joints and the *installed* 6-DOF URDF. It solves offline first.
Run: python3 05_optimisation.py         (DRY RUN)
     python3 05_optimisation.py --execute  (simulation only, after inspection)
"""
import argparse
import numpy as np
from scipy.optimize import least_squares
import rclpy
from kinova_model import KinovaModel
from kinova_ros import KinovaROS


def optimise_xyz(model, q_start, target_xyz):
    """Return (q, converged, evaluations), preserving a near-start solution.

    Optimiser minimises XYZ error plus a small joint-movement penalty.
    This position-only solver does NOT control end-effector orientation.
    """
    q_start = np.asarray(q_start, dtype=float)
    target_xyz = np.asarray(target_xyz, dtype=float)
    if q_start.shape != (6,) or target_xyz.shape != (3,):
        raise ValueError("Expected 6 joint angles and 3 XYZ coordinates")
    if not model.valid_limits(q_start):
        raise ValueError("Starting pose outside URDF joint limits")

    def residual(q):
        # TODO 1: use model.position(q) to calculate XYZ prediction error
        xyz_error = None
        # TODO 2: add a small penalty for moving far from q_start
        regularisation = None
        if xyz_error is None or regularisation is None:
            raise NotImplementedError("Complete the optimiser residual TODOs")
        return np.concatenate([xyz_error, regularisation])

    # TODO 3: call scipy.optimize.least_squares with `residual`, starting
    # joint angles and `bounds=(model.lower, model.upper)`.
    # Use max_nfev=250 and the solver tolerances provided by your instructor.
    result = None
    if result is None:
        raise NotImplementedError("Complete the least_squares TODO")
    position_error = np.linalg.norm(model.position(result.x) - target_xyz)
    return result.x, bool(result.success and position_error < 0.005), result.nfev


def main():
    parser = argparse.ArgumentParser(description="Six-joint position-only IK using SciPy")
    parser.add_argument("--x", type=float)
    parser.add_argument("--y", type=float)
    parser.add_argument("--z", type=float)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    explicit = [args.x, args.y, args.z]
    if any(v is not None for v in explicit) and not all(v is not None for v in explicit):
        parser.error("Specify all of --x, --y and --z, or none")

    rclpy.init()
    arm = KinovaROS()
    try:
        q0 = arm.wait_for_state()
        model = KinovaModel()
        if args.x is None:
            q_nearby = q0.copy()
            q_nearby[0] += 0.10
            q_nearby[1] += 0.08
            if not model.valid_limits(q_nearby):
                raise ValueError("Demo pose exceeds joint limits")
            target = model.position(q_nearby)
        else:
            target = np.array(explicit, dtype=float)

        q, converged, nfev = optimise_xyz(model, q0, target)
        err = np.linalg.norm(model.position(q) - target)
        max_step = np.max(np.abs(q-q0))
        print("Start XYZ [m]:", np.round(model.position(q0), 4))
        print("Target XYZ [m]:", np.round(target, 4))
        print("Solved XYZ [m]:", np.round(model.position(q), 4))
        print(f"Error: {err:.5f} m; evaluations: {nfev}; converged: {converged}")
        print("Delta joints [rad]:", np.round(q-q0, 4))
        if not args.execute:
            print("DRY RUN: add --execute after inspecting the result in a clear Gazebo scene")
            return
        if not converged or err > 0.01 or max_step > 0.35 or not model.valid_limits(q):
            raise RuntimeError("Refusing to command: large step, poor IK or joint limits")
        arm.send(q, duration=5)
        reached = arm.wait_for_target(q, timeout=12)
        print("Reached joint target:", reached)
        if not reached:
            raise RuntimeError("Robot did not reach joint target")
        print("Measured XYZ error [m]:", np.linalg.norm(model.position(arm.q)-target))
    finally:
        arm.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
