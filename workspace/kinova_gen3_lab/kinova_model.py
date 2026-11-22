"""URDF-based kinematics for the Kinova Gen3 6-DOF (ROS 2 Humble).

The numerical geometry is extracted from the installed Kinova Xacro: no DH
parameters are hard-coded. This reusable file is supplied COMPLETE to students.
"""
from dataclasses import dataclass
import math
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

JOINTS = tuple(f"joint_{i}" for i in range(1, 7))
BASE = "base_link"
TIP = "end_effector_link"
XACRO = "/opt/ros/humble/share/kortex_description/robots/kinova.urdf.xacro"


@dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray
    lower: float | None
    upper: float | None


def _vector(el, key, fallback):
    return np.fromstring(el.get(key, fallback), sep=" ", dtype=float)


def rpy_rotation(rpy):
    """URDF fixed-axis roll/pitch/yaw: Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_rotation(axis, angle):
    """Rodrigues' rotation around a URDF joint axis."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + s * K + (1.0 - c) * (K @ K)


def make_transform(rotation=None, translation=None):
    T = np.eye(4)
    if rotation is not None:
        T[:3, :3] = rotation
    if translation is not None:
        T[:3, 3] = translation
    return T


def joint_transform(joint, angle=0.0):
    """Parent-link to child-link transform for a URDF joint."""
    T_origin = make_transform(rpy_rotation(joint.rpy), joint.xyz)
    if joint.kind in ("continuous", "revolute"):
        T_motion = make_transform(axis_rotation(joint.axis, angle))
    elif joint.kind == "fixed":
        T_motion = np.eye(4)
    elif joint.kind == "prismatic":
        T_motion = make_transform(translation=joint.axis * angle)
    else:
        raise ValueError(f"Unsupported joint type {joint.kind}: {joint.name}")
    return T_origin @ T_motion


class KinovaModel:
    def __init__(self, urdf_text=None):
        if urdf_text is None:
            result = subprocess.run(
                ["xacro", XACRO, "name:=gen3", "arm:=gen3", "dof:=6",
                 "sim_ignition:=true", "gripper:="],
                capture_output=True, text=True, check=True,
            )
            urdf_text = result.stdout
        root = ET.fromstring(urdf_text)
        by_child = {}
        for element in root.findall("joint"):
            parent = element.find("parent").get("link")
            child = element.find("child").get("link")
            origin = element.find("origin")
            axis = element.find("axis")
            limits = element.find("limit")
            by_child[child] = Joint(
                name=element.get("name"),
                kind=element.get("type"),
                parent=parent,
                child=child,
                xyz=_vector(origin, "xyz", "0 0 0") if origin is not None else np.zeros(3),
                rpy=_vector(origin, "rpy", "0 0 0") if origin is not None else np.zeros(3),
                axis=_vector(axis, "xyz", "1 0 0") if axis is not None else np.array([1., 0., 0.]),
                lower=float(limits.get("lower")) if limits is not None and limits.get("lower") is not None else None,
                upper=float(limits.get("upper")) if limits is not None and limits.get("upper") is not None else None,
            )
        chain = []
        link = TIP
        while link != BASE:
            if link not in by_child:
                raise ValueError(f"No URDF path from {BASE} to {TIP}; stopped at {link}")
            joint = by_child[link]
            chain.append(joint)
            link = joint.parent
        self.chain = list(reversed(chain))
        names_in_chain = [j.name for j in self.chain if j.kind != "fixed"]
        if names_in_chain != list(JOINTS):
            raise ValueError(f"Unexpected arm joints: {names_in_chain}")
        self.lower = np.array([next(j.lower if j.lower is not None else -np.inf
                                    for j in self.chain if j.name == name) for name in JOINTS])
        self.upper = np.array([next(j.upper if j.upper is not None else np.inf
                                    for j in self.chain if j.name == name) for name in JOINTS])

    def fk_transform(self, q):
        """Return homogeneous transform base_link -> end_effector_link."""
        q = np.asarray(q, dtype=float)
        if q.shape != (6,):
            raise ValueError("Expected six joint angles in joint_1 ... joint_6 order")
        angles = dict(zip(JOINTS, q))
        T = np.eye(4)
        for joint in self.chain:
            T = T @ joint_transform(joint, angles.get(joint.name, 0.0))
        return T

    def position(self, q):
        return self.fk_transform(q)[:3, 3].copy()

    def valid_limits(self, q):
        q = np.asarray(q)
        return bool(np.all(q >= self.lower - 1e-9) and np.all(q <= self.upper + 1e-9))
