import copy

import numpy as np
import pinocchio as pin
from robot_model.robot_wrapper import RobotWrapper
from std_srvs.srv import SetBool

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node

from pi3hat_moteus_int_msgs.msg import JointsCommand, JointsStates


class IKController(Node):
    def __init__(self):
        super().__init__('ik_controller')

        # Declare all parameters
        self.declare_parameter('robot_name', 'mulinex')
        self.declare_parameter('rate', 100.0)
        self.declare_parameter('ik_iterations', 10)
        self.declare_parameter('ik_gain', 2.0)
        self.declare_parameter('ik_alpha', 0.75)
        self.declare_parameter('pos_limit_x', 0.05)
        self.declare_parameter('pos_limit_y', 0.02)
        self.declare_parameter('pos_limit_z', 0.05)
        self.declare_parameter('ang_limit_roll', 0.3)
        self.declare_parameter('ang_limit_pitch', 0.3)
        self.declare_parameter('ang_limit_yaw', 0.15)
        self.declare_parameter('kp_scale', 1.0)
        self.declare_parameter('kd_scale', 1.0)

        # Read parameters
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        rate = self.get_parameter('rate').get_parameter_value().double_value
        self.ik_iterations = self.get_parameter('ik_iterations').get_parameter_value().integer_value
        self.ik_gain = self.get_parameter('ik_gain').get_parameter_value().double_value
        self.ik_alpha = self.get_parameter('ik_alpha').get_parameter_value().double_value
        self.pos_limits = np.array(
            [
                self.get_parameter('pos_limit_x').get_parameter_value().double_value,
                self.get_parameter('pos_limit_y').get_parameter_value().double_value,
                self.get_parameter('pos_limit_z').get_parameter_value().double_value,
            ]
        )
        self.ang_limits = np.array(
            [
                self.get_parameter('ang_limit_roll').get_parameter_value().double_value,
                self.get_parameter('ang_limit_pitch').get_parameter_value().double_value,
                self.get_parameter('ang_limit_yaw').get_parameter_value().double_value,
            ]
        )
        self.kp_scale = self.get_parameter('kp_scale').get_parameter_value().double_value
        self.kd_scale = self.get_parameter('kd_scale').get_parameter_value().double_value

        # Subscriptions
        self.create_subscription(
            Pose, '/ik_controller/base_pose', self.base_pose_command_callback, 1
        )
        self.create_subscription(
            JointsStates, '/omni_controller/joints_state', self.joint_states_callback, 1
        )
        self.create_subscription(
            JointsCommand,
            '/omni_controller/debug/joints_command',
            self.joints_command_callback,
            1,
        )

        # Publisher
        self.joint_command_pub = self.create_publisher(
            JointsCommand, '/omni_controller/joints_reference', 1
        )

        # Reinitialize service
        self.create_service(SetBool, '/ik_controller/reinitialize_srv', self.reinitialize_callback)

        # Activate service
        self.create_service(SetBool, '/ik_controller/activate_srv', self.activate_callback)

        # Timer
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        # Internal variables
        self.active = False
        self.initialized = False
        self.robot_model = RobotWrapper(self.robot_name)

        n_joints = len(self.robot_model.joint_names)
        self.n_joints_per_leg = self.robot_model.n_joints_per_leg

        self.position_command = np.zeros(3)
        self.orientation_command = pin.Quaternion(1, 0, 0, 0)

        self.joint_positions = np.zeros(n_joints)
        self.joint_ref = np.zeros(n_joints)
        # Last commanded positions echoed by omni_controller. Used to seed the IK
        # on activation so we don't snapshot the gravity-sagged measured pose.
        self.commanded_positions = np.zeros(n_joints)
        self.commanded_received = False

        self.initial_feet_pos = []

        self.get_logger().info(
            f'IK controller initialized for {self.robot_name} '
            f'({n_joints} joints, {self.n_joints_per_leg} per leg)'
        )

    def reinitialize_callback(self, request, response):
        self.initialized = False
        self.get_logger().info('IK controller reinitialization requested')
        response.success = True
        response.message = 'IK controller will reinitialize from next joint states'
        return response

    def activate_callback(self, request, response):
        if request.data:
            self.initialized = False  # Force reinitialization from current joint positions
            self.active = True
            self.get_logger().info('IK controller activated')
            response.message = 'IK controller activated, will reinitialize from next joint states'
        else:
            self.active = False
            self.get_logger().info('IK controller deactivated')
            response.message = 'IK controller deactivated'
        response.success = True
        return response

    def joint_states_callback(self, msg: JointsStates):
        # Reorder joints to match the order in all_robots.yaml
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.robot_model.joint_names:
                idx = self.robot_model.joint_names.index(joint_name)
                self.joint_positions[idx] = msg.position[i]

        # Initialize on first valid message (only when active).
        # Seed from the last commanded pose, not the measured pose: the legs
        # sag a few mrad below the command under gravity, and snapshotting the
        # sagged config would drop the body each time IK is (re)activated.
        if self.active and not self.initialized and len(msg.position) > 0:
            if not self.commanded_received:
                self.get_logger().warn(
                    'IK activated but no joints_command received yet; deferring initialization',
                    throttle_duration_sec=1.0,
                )
                return
            self.initialized = True
            self.joint_ref = self.commanded_positions.copy()
            self.robot_model.compute_kinematics(self.commanded_positions)
            self.initial_feet_pos = copy.deepcopy(self.robot_model.get_feet_pos())
            self.get_logger().info('IK controller initialized from commanded joint positions')

    def joints_command_callback(self, msg: JointsCommand):
        # Mirror the joint_states reordering. Skip NaN entries (wheels in IK
        # mode publish NaN positions) and entries for joints we don't model.
        for i, joint_name in enumerate(msg.name):
            if joint_name not in self.robot_model.joint_names:
                continue
            if i >= len(msg.position) or np.isnan(msg.position[i]):
                continue
            idx = self.robot_model.joint_names.index(joint_name)
            self.commanded_positions[idx] = msg.position[i]
            self.commanded_received = True

    def base_pose_command_callback(self, msg: Pose):
        self.position_command = np.array([msg.position.x, msg.position.y, msg.position.z])
        self.orientation_command = pin.Quaternion(
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        )

    def timer_callback(self):
        if not self.active or not self.initialized:
            return

        # Clamp position command
        position = np.clip(self.position_command, -self.pos_limits, self.pos_limits)

        # Clamp orientation command using axis-angle representation
        R_cmd = self.orientation_command.matrix()
        aa = pin.log3(R_cmd)
        aa = np.clip(aa, -self.ang_limits, self.ang_limits)
        R_clamped = pin.exp3(aa)
        orientation = pin.Quaternion(R_clamped)

        # Compute desired base pose as SE3
        desired_pose = pin.SE3(orientation.matrix(), position)

        # Compute feet targets: feet are fixed in world, base moves
        # feet_target[i] = desired_pose.inverse().act(initial_feet_pos[i])
        feet_pos_target = []
        for pos in self.initial_feet_pos:
            feet_pos_target.append(desired_pose.inverse().act(pos))

        # Iterative inverse kinematics (Newton-Raphson)
        beta = 0.7  # 0..1, higher = follow measured more
        joint_ref = beta * self.joint_positions + (1.0 - beta) * self.joint_ref
        n_jpl = self.n_joints_per_leg  # joints per leg (2 for mulinex)

        for it in range(self.ik_iterations):
            self.robot_model.compute_kinematics(joint_ref)
            J_feet = self.robot_model.get_feet_jacobian()
            feet_pos = self.robot_model.get_feet_pos()

            for i in range(len(feet_pos)):
                residual = feet_pos_target[i] - feet_pos[i]
                delta_q = np.linalg.pinv(J_feet[i]).dot(residual)
                joint_ref[n_jpl * i : n_jpl * (i + 1)] += (
                    delta_q[6 + n_jpl * i : 6 + n_jpl * (i + 1)] * self.ik_gain * self.ik_alpha**it
                )

        self.joint_ref = joint_ref

        # Publish JointsCommand
        msg = JointsCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.robot_model.joint_names
        msg.position = joint_ref.tolist()
        msg.velocity = [0.0] * len(self.robot_model.joint_names)
        msg.effort = [0.0] * len(self.robot_model.joint_names)
        msg.kp_scale = [self.kp_scale] * len(self.robot_model.joint_names)
        msg.kd_scale = [self.kd_scale] * len(self.robot_model.joint_names)

        self.joint_command_pub.publish(msg)


def main(args=None):
    np.set_printoptions(precision=3, linewidth=300)
    rclpy.init(args=args)
    node = IKController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
