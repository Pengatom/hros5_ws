#!/usr/bin/env python3
import subprocess
from typing import Optional

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class ResetWorldService(Node):
    def __init__(self):
        super().__init__('reset_world_service')
        self.world_name = self.declare_parameter('world_name', 'empty').get_parameter_value().string_value
        self.create_service(Trigger, 'reset_world', self.reset_callback)
        self.get_logger().info(f'Reset service ready on /reset_world targeting world "{self.world_name}"')

    def reset_callback(self, request, response):
        world = self.world_name or 'empty'
        cmd = [
            'gz', 'service',
            '-s', f'/world/{world}/control',
            '--reqtype', 'gz.msgs.WorldControl',
            '--reptype', 'gz.msgs.Boolean',
            '--req', 'reset: {time_only: true}'
        ]
        try:
            completed = subprocess.run(cmd, check=False, capture_output=True, text=True)
            ok = completed.returncode == 0
            response.success = ok
            response.message = completed.stdout.strip() if ok else completed.stderr.strip()
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
        return response


def main(args: Optional[list] = None):
    rclpy.init(args=args)
    node = ResetWorldService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
