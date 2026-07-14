#!/usr/bin/env python3
"""Read-only preflight checks for the PICO/Isaac Lab integration."""

from __future__ import annotations

import argparse
import importlib.metadata
import importlib.util
import json
import sys
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

from pico_isaaclab.joint_contract import inspect_urdf_contract


def package_version(name: str) -> str | None:
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--isaaclab-root", type=Path, default=Path("/srv/IsaacLabCode"),
    )
    args = parser.parse_args()
    root = args.isaaclab_root.expanduser().resolve()
    urdf = root / "BRX042501" / "BRX042501_wheel_4cams.urdf"
    result: dict[str, object] = {
        "isaaclab_root": str(root),
        "isaaclab_version_file": None,
        "urdf": str(urdf),
        "urdf_ok": False,
        "rclpy_importable": importlib.util.find_spec("rclpy") is not None,
        "lerobot_package_version": package_version("lerobot"),
    }
    version_file = root / "VERSION"
    if version_file.is_file():
        result["isaaclab_version_file"] = version_file.read_text(encoding="utf-8").strip()
    if urdf.is_file():
        report = inspect_urdf_contract(urdf)
        result.update(
            urdf_ok=True,
            joint_count=report.joint_count,
            non_fixed_joint_count=report.non_fixed_joint_count,
            stereo_baseline_m=report.head_stereo_baseline_m,
        )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
