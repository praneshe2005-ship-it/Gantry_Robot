#!/usr/bin/env python3
"""Keyboard x/y/z straight-line jogging of the end effector through MoveIt.

  x / X : +step / -step along X
  y / Y : +step / -step along Y
  z / Z : +step / -step along Z
  q     : quit

Each key press asks MoveIt for a straight-line (Cartesian) path from the
current end-effector pose to a point `step` metres further along the chosen
axis, then executes it. The tool orientation is kept the same.
"""
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

KEY_TO_DIRECTION = {
    'x': (1.0, 0.0, 0.0), 'X': (-1.0, 0.0, 0.0),
    'y': (0.0, 1.0, 0.0), 'Y': (0.0, -1.0, 0.0),
    'z': (0.0, 0.0, 1.0), 'Z': (0.0, 0.0, -1.0),
}

MOVEIT_SUCCESS = 1  # moveit_msgs/MoveItErrorCodes.SUCCESS
MIN_FRACTION = 0.99  # refuse to move unless (almost) the whole line is reachable


class CartesianKeys(Node):
    def __init__(self):
        super().__init__('cartesian_keys')
        self.declare_parameter('step', 0.01)            # metres per key press
        self.declare_parameter('group', 'arm')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tip_frame', 'link6')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.joint_state = None
        self.create_subscription(JointState, 'joint_states', self._on_joint_state, 10)

        self.cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')

    def _on_joint_state(self, msg):
        self.joint_state = msg

    @staticmethod
    def _wait(future, timeout):
        end = time.time() + timeout
        while not future.done():
            if time.time() > end:
                return False
            time.sleep(0.01)
        return True

    def wait_for_moveit(self):
        if not self.cartesian_client.wait_for_service(timeout_sec=10.0):
            print('ERROR: /compute_cartesian_path not found. Is demo.launch.py running?')
            return False
        if not self.execute_client.wait_for_server(timeout_sec=10.0):
            print('ERROR: /execute_trajectory not found. Is demo.launch.py running?')
            return False
        return True

    def move_step(self, direction):
        base = self.get_parameter('base_frame').value
        tip = self.get_parameter('tip_frame').value
        group = self.get_parameter('group').value
        step = self.get_parameter('step').value

        if self.joint_state is None:
            print('No /joint_states received yet - is the controller running?')
            return

        # 1. Where is the tool right now?
        try:
            tf = self.tf_buffer.lookup_transform(base, tip, Time())
        except TransformException as exc:
            print(f'Could not read pose of {tip} in {base}: {exc}')
            return

        # 2. Target = current position + one step along the chosen axis,
        #    same orientation as now.
        target = Pose()
        target.position.x = tf.transform.translation.x + direction[0] * step
        target.position.y = tf.transform.translation.y + direction[1] * step
        target.position.z = tf.transform.translation.z + direction[2] * step
        target.orientation = tf.transform.rotation

        # 3. Ask MoveIt for a straight-line path.
        request = GetCartesianPath.Request()
        request.header.frame_id = base
        request.start_state = RobotState()
        request.start_state.joint_state = self.joint_state
        request.group_name = group
        request.link_name = tip
        request.waypoints = [target]
        request.max_step = 0.005          # check the line every 5 mm
        request.jump_threshold = 0.0      # 0 = jump checking disabled
        request.avoid_collisions = True

        future = self.cartesian_client.call_async(request)
        if not self._wait(future, 5.0):
            print('Timed out waiting for compute_cartesian_path')
            return
        response = future.result()

        if response.error_code.val != MOVEIT_SUCCESS:
            print(f'MoveIt could not plan (error code {response.error_code.val})')
            return
        if response.fraction < MIN_FRACTION:
            print(f'Blocked: only {response.fraction * 100:.0f}% of that line is reachable '
                  '(joint limit, singularity or collision). Not moving.')
            return
        if not response.solution.joint_trajectory.points:
            print('MoveIt returned an empty trajectory. Not moving.')
            return

        # 4. Execute the planned trajectory.
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        send_future = self.execute_client.send_goal_async(goal)
        if not self._wait(send_future, 5.0):
            print('Timed out sending the trajectory')
            return
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            print('Trajectory was rejected by MoveIt')
            return
        result_future = goal_handle.get_result_async()
        if not self._wait(result_future, 30.0):
            print('Timed out waiting for the motion to finish')
            return
        result = result_future.result().result
        if result.error_code.val == MOVEIT_SUCCESS:
            print('  done')
        else:
            print(f'Execution failed (error code {result.error_code.val})')


def main():
    rclpy.init()
    node = CartesianKeys()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    if not node.wait_for_moveit():
        rclpy.shutdown()
        return

    step = node.get_parameter('step').value
    print(f'Ready. Step = {step * 1000:.0f} mm.')
    print('  x / X : +/- along X     y / Y : +/- along Y     z / Z : +/- along Z')
    print('  q     : quit')

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            key = sys.stdin.read(1)
            if key == 'q':
                break
            direction = KEY_TO_DIRECTION.get(key)
            if direction is None:
                continue
            print(f'{key}: moving...')
            node.move_step(direction)
            # Drop any keys pressed while the arm was moving, so they don't queue up.
            termios.tcflush(fd, termios.TCIFLUSH)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
