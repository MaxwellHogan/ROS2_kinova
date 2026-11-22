"""Small ROS 2 helper. Supplied complete so kinematics exercises focus on maths.

Use ONLY with the working Gazebo simulation; direct trajectories are not
collision checked. The helper orders joint values by name and waits for feedback.
"""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from kinova_model import JOINTS


class KinovaROS(Node):
    def __init__(self):
        super().__init__("kinova_lab_python")
        self.latest = {}
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self._state_callback, 10)
        self.publisher = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)

    def _state_callback(self, message):
        self.latest.update(zip(message.name, message.position))

    @property
    def q(self):
        if not all(name in self.latest for name in JOINTS):
            return None
        return np.array([self.latest[name] for name in JOINTS], dtype=float)

    def wait_for_state(self, timeout=15):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.q is not None:
                return self.q.copy()
        raise TimeoutError("No six-joint state received. Is Gazebo running?")

    def send(self, target, duration=4):
        target = np.asarray(target, dtype=float)
        if target.shape != (6,) or not np.all(np.isfinite(target)):
            raise ValueError("Expected finite joint_1..joint_6 target angles")
        if self.publisher.get_subscription_count() == 0:
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and self.publisher.get_subscription_count() == 0:
                rclpy.spin_once(self, timeout_sec=0.1)
            if self.publisher.get_subscription_count() == 0:
                raise RuntimeError("Joint trajectory controller is not subscribing")
        msg = JointTrajectory()
        msg.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = target.tolist()
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        msg.points = [point]
        self.publisher.publish(msg)

    def wait_for_target(self, target, tolerance=0.025, timeout=12):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            q = self.q
            if q is not None and np.max(np.abs(q - target)) <= tolerance:
                return True
        return False
