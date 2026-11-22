"""EXERCISE 2: send a ROS 2 JointTrajectory to the simulated Kinova arm.

Move the first joint a small amount (degrees -> radians), leaving the other
five joint positions unchanged. Only execute in Gazebo.
"""
import numpy as np
import time
import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from kinova_ros import KinovaROS
from kinova_model import JOINTS, KinovaModel


def main():
    rclpy.init()
    arm = KinovaROS()
    try:
        current = arm.wait_for_state()
        print("Current angles (rad):", np.round(current, 3))
        model = KinovaModel()

        # TODO 1: create target as a copy of current joint angles
        target = None

        # TODO 2: increase joint_1 by 10 degrees (convert to radians)
        # target[0] += ...

        if target is None:
            raise NotImplementedError("Complete the target angle TODOs")
        if not model.valid_limits(target) or np.max(np.abs(target - current)) > 0.35:
            raise ValueError("Target exceeds joint limits or classroom step size")

        # TODO 3: construct a JointTrajectory with JOINTS as joint_names
        trajectory = None

        # TODO 4: construct a JointTrajectoryPoint; set positions and
        # time_from_start to 4 seconds, then add it to trajectory.points

        if trajectory is None or not trajectory.points:
            raise NotImplementedError("Complete the trajectory message TODOs")

        # ROS interface publishes the message constructed by the student
        deadline = time.monotonic() + 5.0
        while arm.publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(arm, timeout_sec=0.1)
        if arm.publisher.get_subscription_count() == 0:
            raise TimeoutError("Joint trajectory controller is not subscribing")
        arm.publisher.publish(trajectory)
        completed = arm.wait_for_target(target, timeout=10)
        print("Goal reached:", completed)
        print("Final angles (rad):", np.round(arm.q, 3))
    finally:
        arm.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
