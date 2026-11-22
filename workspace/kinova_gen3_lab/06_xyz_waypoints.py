"""OPTIONAL DEMO: adapted version of the original 7dof.py target loop.

Works on six joints with ROS 2. Cartesian waypoints default to *reachable*
nearby XYZ points derived from six-joint configurations in the URDF. Dry-run
by default. NO collision checking: never run on real hardware.

python3 06_xyz_waypoints.py
python3 06_xyz_waypoints.py --execute

To use your own XYZ waypoints, edit `targets` in make_demo_targets().
"""
import argparse
import numpy as np
import rclpy
from kinova_model import KinovaModel
from kinova_ros import KinovaROS

# Import the numerical-Jacobian IK solution from Exercise 4.
from pathlib import Path
import importlib.util


def load_ik_solver():
    base = Path(__file__).parent
    instructor_solution = base / "solutions" / "04_inverse_kinematics_solution.py"
    path = instructor_solution if instructor_solution.exists() else base / "04_inverse_kinematics.py"
    spec = importlib.util.spec_from_file_location("kinova_exercise4", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.solve_ik


def make_demo_targets(model, q0):
    """Small excursions in six-joint space transformed to XYZ targets.

    This is an initial set of attainable XYZ targets, NOT a collision-free path.
    Replace with explicit XYZ coordinates in metres after verifying reachability.
    """
    offsets = np.array([
        [0.00, 0.00, 0.00, 0.00, 0.00, 0.00],
        [0.08, 0.06, 0.00, 0.00, 0.00, 0.00],
        [0.10, 0.03, 0.04, 0.00, 0.00, 0.00],
        [0.03, 0.08, 0.04, 0.00, 0.00, 0.00],
        [0.00, 0.00, 0.00, 0.00, 0.00, 0.00],
    ])
    targets = []
    for offset in offsets:
        q = q0 + offset
        if not model.valid_limits(q):
            raise ValueError("Demo joint configuration exceeds the URDF limits")
        targets.append(model.position(q))
    return targets


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", help="Send checked trajectories to Gazebo")
    args = parser.parse_args()
    rclpy.init()
    arm = KinovaROS()
    try:
        q0 = arm.wait_for_state()
        model = KinovaModel()
        solve_ik = load_ik_solver()
        targets = make_demo_targets(model, q0)
        proposed = []
        previous = q0.copy()
        for index, xyz in enumerate(targets):
            q, converged, iterations = solve_ik(model, previous, xyz)
            error = float(np.linalg.norm(model.position(q) - xyz))
            joint_change = float(np.max(np.abs(q - previous)))
            safe = converged and error < 0.01 and joint_change <= 0.35 and model.valid_limits(q)
            print(f"{index}: target XYZ={np.round(xyz, 4)}, "
                  f"FK error={error:.5f} m, max joint change={joint_change:.3f} rad, "
                  f"iterations={iterations}, checked={safe}")
            if not safe:
                raise RuntimeError("Rejected one waypoint: no trajectories sent")
            proposed.append((xyz, q.copy()))
            previous = q.copy()

        if not args.execute:
            print("DRY RUN complete; execute only in a clear Gazebo simulation")
            return

        for idx, (xyz, q) in enumerate(proposed):
            print(f"Executing waypoint {idx}: {np.round(xyz, 4)}")
            arm.send(q, duration=5)
            if not arm.wait_for_target(q, timeout=12):
                raise RuntimeError(f"Stopped: joint target not reached at waypoint {idx}")
            actual_xyz = model.position(arm.q)
            print(f"Actual XYZ {np.round(actual_xyz, 4)}, error "
                  f"{np.linalg.norm(actual_xyz-xyz):.5f} m")
    finally:
        arm.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
