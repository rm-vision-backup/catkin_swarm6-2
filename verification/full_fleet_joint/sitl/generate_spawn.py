#!/usr/bin/env python3
"""按 mission_spec 的 available 生成 PX4/Gazebo 出生 launch。

坐标：任务地面网格 + (100,100)，机头朝 Gazebo +Y（与任务 +y 一致）。
端口：mavlink_tcp=4560+(n-1)，udp_gimbal=13030+(n-1)。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from verification.full_fleet_joint.generate_mission import grid_map, load_fleet, uav_num


def build_launch(spec_path: Path, fleet_path: Path) -> str:
    spec = json.loads(Path(spec_path).read_text(encoding="utf-8"))
    fleet = load_fleet(fleet_path)
    grid = grid_map(fleet, spec.get("slot_replacements") or {})
    # 出生点必须用任务包的地面机位（已含 calibration.shift_x/shift_y），
    # 否则未受控的备用机/ B 组会与任务坐标差一个标定原点偏移。
    ground = spec.get("ground_positions") or {}
    available = sorted(spec["available_uavs"], key=uav_num)
    lines = [
        "<launch>",
        "  <!-- generated from %s -->" % Path(spec_path).name,
        '  <arg name="gui" default="false"/>',
        '  <include file="$(find gazebo_ros)/launch/empty_world.launch">',
        '    <arg name="gui" value="$(arg gui)"/>',
        '    <arg name="world_name" value="$(find mavlink_sitl_gazebo)/worlds/empty.world"/>',
        "  </include>",
    ]
    for uav in available:
        number = uav_num(uav)
        identifier = number - 1
        task_id = "A%02d" % number
        if task_id in ground:
            gx, gy = ground[task_id]["x"], ground[task_id]["y"]
        else:
            gx, gy = grid[uav]
        lines.extend(
            [
                '  <group ns="iris_%d">' % identifier,
                '    <include file="$(find px4)/launch/single_vehicle_spawn_xtd.launch">',
                '      <arg name="x" value="%s"/>' % (100.0 + gx),
                '      <arg name="y" value="%s"/>' % (100.0 + gy),
                '      <arg name="z" value="0.5"/>',
                '      <arg name="R" value="0"/><arg name="P" value="0"/>',
                '      <arg name="Y" value="1.57079632679"/>',
                '      <arg name="vehicle" value="iris"/><arg name="sdf" value="iris"/>',
                '      <arg name="ID" value="%d"/><arg name="ID_in_group" value="%d"/>'
                % (identifier, identifier),
                '      <arg name="mavlink_tcp_port" value="%d"/>' % (4560 + identifier),
                '      <arg name="udp_gimbal_port" value="%d"/>' % (13030 + identifier),
                '      <arg name="interactive" value="true"/>',
                "    </include>",
                "  </group>",
            ]
        )
    lines.append("</launch>")
    return "\n".join(lines) + "\n"


def generate(spec_path: Path, out_path: Path, fleet_path: Path) -> Path:
    text = build_launch(Path(spec_path), Path(fleet_path))
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(text, encoding="utf-8")
    return out_path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--spec",
        default=str(HERE.parent / "generated" / "mission_spec.json"),
    )
    parser.add_argument("--fleet", default=str(HERE.parent / "fleet.yaml"))
    parser.add_argument("--out", default=str(HERE / "spawn.launch"))
    args = parser.parse_args(argv)
    out = generate(Path(args.spec), Path(args.out), Path(args.fleet))
    print("已生成 %s" % out.resolve())
    return 0


if __name__ == "__main__":
    sys.exit(main())
