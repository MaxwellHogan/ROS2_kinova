"""EXERCISE 1: read and display all 6 joint angles with rclpy.

Fill in the TODOs and run: python3 01_read_joints.py
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = [f"joint_{i}" for i in range(1, 7)]


class ReadJoints(Node):
    def __init__(self):
        super().__init__("read_kinova_joints")
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.last_display_time = 0.0

    def on_joint_states(self, msg):
        # TODO 1: make a dictionary mapping joint name -> angle in radians
        by_name = None

        # TODO 2: obtain values in JOINTS order (never assume msg.position order)
        # Ignore messages without all six joints.
        angles_rad = None

        # TODO 3: convert each angle into degrees
        angles_deg = None

        if angles_deg is not None:
            # The callback is invoked at sensor frequency; avoid excess output
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_display_time >= 1:
                self.last_display_time = now
                for name, rad, deg in zip(JOINTS, angles_rad, angles_deg):
                    print(f"{name}: {rad:+.3f} rad / {deg:+.1f} deg")
                print("-" * 32)


def main():
    rclpy.init()
    node = ReadJoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
