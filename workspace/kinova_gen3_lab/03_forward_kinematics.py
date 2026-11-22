"""EXERCISE 3: calculate end-effector XYZ by multiplying URDF transforms.

The supplied model reads all real robot geometry from Kinova's Xacro file.
Students fill in the actual forward-kinematics multiplication.
"""
import numpy as np
import rclpy
from kinova_model import KinovaModel, JOINTS, joint_transform
from kinova_ros import KinovaROS


def student_forward_kinematics(model, q):
    angles = dict(zip(JOINTS, q))

    # TODO 1: initialise a 4x4 identity transform
    T = None

    # TODO 2: loop over model.chain (includes fixed and moving joints)
    # For every joint: angle = angles.get(joint.name, 0.0)
    # T = T @ joint_transform(joint, angle)

    # TODO 3: return XYZ from the homogeneous transformation
    return None


def main():
    rclpy.init()
    arm = KinovaROS()
    try:
        q = arm.wait_for_state()
        model = KinovaModel()
        xyz = student_forward_kinematics(model, q)
        if xyz is None:
            raise NotImplementedError("Complete student_forward_kinematics")
        print("Joint positions [rad] :", np.round(q, 4))
        print("Student FK [m]        :", np.round(xyz, 4))
        print("Reference FK [m]      :", np.round(model.position(q), 4))
        print("FK error [m]          :", np.linalg.norm(xyz - model.position(q)))
        print("Compare against TF with: ros2 run tf2_ros tf2_echo base_link end_effector_link")
    finally:
        arm.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
