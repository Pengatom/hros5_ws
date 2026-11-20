#!/usr/bin/env python3
"""Simulated Dynamixel bridge for Gazebo.

Subscribes to the same /hros5/<limb>/target_angles_deg topics as the real *_dxl_node
executables in hros5_dynamixel_bridge, and behaves like them:
  - absolute targets in servo degrees (0 deg = tick 2048)
  - clamped to the absolute servo limits in hros5_dynamixel_joints_with_limits.yaml
  - rate limited to max_step_deg per 20 ms tick (grip: grip_max_step_deg)
  - starts from 0 deg on every joint (the real nodes don't read the present position)
  - logs joint positions on the limb's echo_positions topic

Instead of writing to the bus, it converts to URDF radians with the per-joint direction
in hros5_control/config/joint_directions.yaml and streams the whole body to the JointTrajectoryController.
The unchanged hros5_teleop nodes therefore drive the simulated robot the same way they
drive the real one.
"""
import math
import os
import xml.etree.ElementTree as ET

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float32MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TICKS_PER_DEG = 4096.0 / 360.0

# Topic, echo topic and message layout of each real DXL node (hros5_dynamixel_bridge/src).
LIMBS = {
    'head': ('/hros5/head/target_angles_deg', '/hros5/head/echo_positions',
             ['HeadYaw', 'HeadPitch']),
    'left_arm': ('/hros5/left_arm/target_angles_deg', '/hros5/left_arm/echo_positions',
                 ['LShoulderPitch', 'LShoulderRoll', 'LElbowPitch', 'LWrist', 'LGrip']),
    'right_arm': ('/hros5/right_arm/target_angles_deg', '/hros5/right_arm/echo_positions',
                  ['RShoulderPitch', 'RShoulderRoll', 'RElbowPitch', 'RWrist', 'RGrip']),
    'left_leg': ('/hros5/left_leg/target_angles_deg', '/hros5/left_leg/echo_positions',
                 ['LHipYaw', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll']),
    'right_leg': ('/hros5/right_leg/target_angles_deg', '/hros5/right_leg/echo_positions',
                  ['RHipYaw', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']),
    'left_hand': ('/hros5/left_hand/target_angles_deg', None, ['LGrip', 'LWrist']),
    'right_hand': ('/hros5/right_hand/target_angles_deg', None, ['RGrip', 'RWrist']),
}
GRIPS = {'LGrip', 'RGrip'}


def clip(x, lo, hi):
    return max(lo, min(hi, x))


class SimDxlBridge(Node):
    def __init__(self):
        super().__init__('sim_dxl_bridge')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('max_step_deg', 2.0)
        self.declare_parameter('grip_max_step_deg', 3.0)
        self.declare_parameter('trajectory_topic', '/whole_body_controller/joint_trajectory')
        self.declare_parameter('servo_package', 'hros5_control')
        self.declare_parameter('servo_limits_file', 'config/hros5_dynamixel_joints_with_limits.yaml')
        self.declare_parameter('servo_joints_dir', 'config/joints')
        self.declare_parameter('joint_directions_file', 'config/joint_directions.yaml')

        self.servo = self._load_servo_config()
        share = get_package_share_directory(self.get_parameter('servo_package').value)
        with open(os.path.join(share, self.get_parameter('joint_directions_file').value)) as f:
            self.direction = {k: int(v) for k, v in yaml.safe_load(f)['direction'].items()}

        self.joints = sorted({n for _, _, names in LIMBS.values() for n in names})
        missing = [n for n in self.joints if n not in self.servo or n not in self.direction]
        if missing:
            raise RuntimeError(f'No servo limits or direction for: {missing}')

        self.target = {n: 0.0 for n in self.joints}   # servo degrees, like the real nodes
        self.current = {n: 0.0 for n in self.joints}
        self.measured = {}
        self.active = False

        for limb, (topic, echo_topic, _) in LIMBS.items():
            self.create_subscription(Float32MultiArray, topic,
                                     lambda m, limb=limb: self._on_command(limb, m), 10)
            if echo_topic:
                self.create_subscription(Empty, echo_topic,
                                         lambda m, limb=limb: self._on_echo(limb), 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        # URDF joint limits: commands are kept urdf_margin_deg inside them. A joint driven exactly
        # onto its URDF limit can stick there in Gazebo/ros2_control (seen on the wrists).
        self.declare_parameter('urdf_margin_deg', 1.0)
        self.urdf_limits = {}
        self.create_subscription(
            String, '/robot_description', self._on_robot_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub = self.create_publisher(
            JointTrajectory, self.get_parameter('trajectory_topic').value, 10)
        self.dt = 1.0 / self.get_parameter('rate_hz').value
        self.create_timer(self.dt, self._update)   # follows sim time when use_sim_time is set

        self.get_logger().info(
            f'Simulating DXL nodes for {len(self.joints)} joints. Waiting for the first '
            f'/hros5/<limb>/target_angles_deg command before taking over the controller.')

    def _load_servo_config(self):
        share = get_package_share_directory(self.get_parameter('servo_package').value)
        limits_path = os.path.join(share, self.get_parameter('servo_limits_file').value)
        servo = {}
        with open(limits_path) as f:
            for j in yaml.safe_load(f)['joints']:
                servo[j['name']] = {
                    'id': j['id'],
                    'min': j['cw_limit'] / TICKS_PER_DEG - 180.0,
                    'max': j['ccw_limit'] / TICKS_PER_DEG - 180.0,
                    'offset': float(j.get('offset_deg', 0.0)),
                }
        joints_dir = os.path.join(share, self.get_parameter('servo_joints_dir').value)
        if os.path.isdir(joints_dir):
            for fn in sorted(os.listdir(joints_dir)):
                if fn.endswith('.yaml'):
                    with open(os.path.join(joints_dir, fn)) as f:
                        for j in (yaml.safe_load(f) or {}).get('joints', []):
                            if j['name'] in servo and 'offset_deg' in j:
                                servo[j['name']]['offset'] = float(j['offset_deg'])
        return servo

    def _on_command(self, limb, msg):
        names = LIMBS[limb][2]
        if len(msg.data) < len(names):
            self.get_logger().warn(
                f'{limb}: expected {len(names)} values, got {len(msg.data)}', throttle_duration_sec=5.0)
            return
        for name, cmd in zip(names, msg.data):
            s = self.servo[name]
            # Same as the real nodes: the servo goes to cmd + offset, clamped to its hard limits.
            self.target[name] = clip(cmd + s['offset'], s['min'], s['max']) - s['offset']
        if not self.active:
            self.active = True
            self.get_logger().info(f'First command from {limb}: now driving the robot.')

    def _on_robot_description(self, msg):
        margin = math.radians(self.get_parameter('urdf_margin_deg').value)
        limits = {}
        for j in ET.fromstring(msg.data).findall('joint'):
            lim = j.find('limit')
            if j.get('type') == 'revolute' and lim is not None and j.get('name') in self.direction:
                limits[j.get('name')] = (float(lim.get('lower')) + margin, float(lim.get('upper')) - margin)
        self.urdf_limits = limits
        # Point out where the URDF allows less than the servo (the sim then stops short of the servo target).
        narrower = []
        for name, (lo, hi) in limits.items():
            s, d = self.servo[name], self.direction[name]
            servo_urdf = sorted((math.radians(d * s['min']), math.radians(d * s['max'])))
            # URDF limits sit >= 2 deg inside the servo limits by design; only report real gaps.
            if lo > servo_urdf[0] + math.radians(5) or hi < servo_urdf[1] - math.radians(5):
                narrower.append(f"{name} (URDF {math.degrees(lo):+.0f}..{math.degrees(hi):+.0f} vs servo "
                                f"{math.degrees(servo_urdf[0]):+.0f}..{math.degrees(servo_urdf[1]):+.0f} deg)")
        self.get_logger().info(f'URDF limits loaded for {len(limits)} joints.')
        if narrower:
            self.get_logger().info('URDF range more than 5 deg narrower than the servo range (#66), sim stops at the URDF limit: '
                                   + '; '.join(narrower))

    def _on_joint_states(self, msg):
        self.measured.update(zip(msg.name, msg.position))

    def _on_echo(self, limb):
        parts = []
        for name in LIMBS[limb][2]:
            if name not in self.measured:
                continue
            deg = self.direction[name] * math.degrees(self.measured[name])
            ticks = int(round((deg + self.servo[name]['offset'] + 180.0) * TICKS_PER_DEG))
            parts.append(f"id {self.servo[name]['id']} {name}: {deg:+.1f} deg ({ticks} ticks)")
        self.get_logger().info(f'echo {limb} (sim): ' + '; '.join(parts))

    def _update(self):
        if not self.active:
            return
        max_step = self.get_parameter('max_step_deg').value
        grip_step = self.get_parameter('grip_max_step_deg').value
        for name in self.joints:
            lim = grip_step if name in GRIPS else max_step
            self.current[name] += clip(self.target[name] - self.current[name], -lim, lim)

        traj = JointTrajectory()
        traj.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = []
        for n in self.joints:
            rad = self.direction[n] * math.radians(self.current[n])
            if n in self.urdf_limits:
                rad = clip(rad, *self.urdf_limits[n])
            point.positions.append(rad)
        t = 2.0 * self.dt
        point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
        traj.points = [point]
        self.pub.publish(traj)


def main():
    rclpy.init()
    node = SimDxlBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
