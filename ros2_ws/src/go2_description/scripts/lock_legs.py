#!/usr/bin/env python3
"""
Generate a Phase-1 "rigid standing base" Go2 URDF from the official Unitree
go2_description.

WHAT THIS DOES (and why):
  The official Go2 has 12 revolute leg joints (hip/thigh/calf x 4 legs). In
  Phase 1 we do NOT simulate legged locomotion. We lock every leg joint into a
  fixed standing stance, which makes Gazebo lump the whole robot into a single
  rigid body shaped like a standing Go2. We then drive that body around the
  ground plane with a gazebo_ros planar_move plugin listening on /cmd_vel.

  This is a deliberate, documented simplification: a kinematic driving base,
  not gait control. The /cmd_vel Twist interface created here is the permanent
  control contract for later phases.

STANCE (radians), baked into each joint's origin rpy (originals are all rpy 0):
  hip   = 0.0    (no leg splay)
  thigh = 0.8
  calf  = -1.5

Run:  python3 lock_legs.py <input_original.urdf> <output_go2.urdf>
"""
import sys
import math
import xml.etree.ElementTree as ET

STANCE = {"hip": 0.0, "thigh": 0.8, "calf": -1.5}

# Kinematic constants from the official const.xacro (metres).
THIGH_LEN = 0.213
CALF_LEN = 0.213
FOOT_RADIUS = 0.02

PLANAR_MOVE_GAZEBO = """
  <!-- ===================================================================
       Phase-1 simplified base controller.
       libgazebo_ros_planar_move consumes geometry_msgs/Twist on /cmd_vel and
       slides the (rigid, leg-locked) body holonomically over the ground plane.
       It also publishes nav_msgs/Odometry on /odom and the odom to base TF.
       This is a kinematic driving base, NOT gait control.
       =================================================================== -->
  <gazebo>
    <plugin name="planar_move_controller" filename="libgazebo_ros_planar_move.so">
      <ros>
        <namespace>/</namespace>
        <remapping>cmd_vel:=cmd_vel</remapping>
        <remapping>odom:=odom</remapping>
      </ros>
      <update_rate>50</update_rate>
      <publish_rate>50</publish_rate>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base</robot_base_frame>
      <covariance_x>0.0001</covariance_x>
      <covariance_y>0.0001</covariance_y>
      <covariance_yaw>0.01</covariance_yaw>
    </plugin>
  </gazebo>

  <!-- Modest friction on the lumped body so it rests cleanly on its feet. -->
  <gazebo reference="base">
    <mu1>0.6</mu1>
    <mu2>0.6</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>
"""


def joint_kind(name):
    for kind in ("hip", "thigh", "calf"):
        if name.endswith(f"_{kind}_joint"):
            return kind
    return None


def lock_legs(in_path, out_path):
    tree = ET.parse(in_path)
    root = tree.getroot()

    locked = 0
    for joint in root.findall("joint"):
        if joint.get("type") != "revolute":
            continue
        kind = joint_kind(joint.get("name", ""))
        if kind is None:
            continue

        angle = STANCE[kind]
        joint.set("type", "fixed")

        # Bake the stance angle into the joint origin. Hip axis is x (roll);
        # thigh and calf axes are y (pitch). Originals are rpy 0, so we just set
        # the rotation about the joint's axis.
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin")
            origin.set("xyz", "0 0 0")
        if kind == "hip":
            origin.set("rpy", f"{angle} 0 0")
        else:  # thigh, calf rotate about y
            origin.set("rpy", f"0 {angle} 0")

        # A fixed joint has no axis or limit; drop them.
        for tag in ("axis", "limit"):
            child = joint.find(tag)
            if child is not None:
                joint.remove(child)
        locked += 1

    # Append the Gazebo planar_move + friction blocks before </robot>.
    extra = ET.fromstring(f"<wrap>{PLANAR_MOVE_GAZEBO}</wrap>")
    for el in list(extra):
        root.append(el)

    ET.indent(tree, space="  ")
    # No XML encoding declaration: robot_state_publisher republishes this string
    # verbatim on /robot_description, and gazebo_ros spawn_entity.py (lxml) rejects
    # a str that carries an encoding declaration.
    tree.write(out_path, encoding="utf-8", xml_declaration=False)

    # Report standing foot height so the launch file can spawn at the right z.
    t, c = STANCE["thigh"], STANCE["calf"]
    foot_drop = THIGH_LEN * math.cos(t) + CALF_LEN * math.cos(t + c) + FOOT_RADIUS
    print(f"Locked {locked} leg joints into stance {STANCE}")
    print(f"Computed foot drop below base: {foot_drop:.3f} m "
          f"-> spawn base at z ~= {foot_drop:.2f}")
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(f"usage: {sys.argv[0]} <input.urdf> <output.urdf>")
    lock_legs(sys.argv[1], sys.argv[2])
