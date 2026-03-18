"""Helpers for preparing the exported URDF for PyBullet."""

from pathlib import Path
from xml.etree import ElementTree as ET

from .joint_map import URDF_JOINT_LIMITS_DEG

PACKAGE_PREFIX = "package://drumrobot_RL_urdf/"


def build_runtime_urdf(source_urdf: Path, output_dir: Path) -> Path:
    """
    Build a runtime-only URDF that PyBullet can load directly.

    The checked-in URDF stays untouched. We only rewrite mesh paths and
    repair zeroed joint limits in a generated copy.
    """
    source_urdf = source_urdf.resolve()
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    package_root = source_urdf.parent.parent
    tree = ET.parse(source_urdf)
    root = tree.getroot()

    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith(PACKAGE_PREFIX):
            relative_path = filename[len(PACKAGE_PREFIX):]
            mesh.set("filename", str((package_root / relative_path).resolve()))

    for joint in root.findall("joint"):
        joint_name = joint.get("name", "")
        if joint_name not in URDF_JOINT_LIMITS_DEG:
            continue

        limit = joint.find("limit")
        if limit is None:
            limit = ET.SubElement(joint, "limit")

        lower_deg, upper_deg = URDF_JOINT_LIMITS_DEG[joint_name]
        limit.set("lower", str(_deg_to_rad(lower_deg)))
        limit.set("upper", str(_deg_to_rad(upper_deg)))
        limit.set("effort", "50")
        limit.set("velocity", "2.5")

    output_path = output_dir / f"{source_urdf.stem}.pybullet.urdf"
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return output_path


def _deg_to_rad(angle_deg: float) -> float:
    return angle_deg * 3.141592653589793 / 180.0
