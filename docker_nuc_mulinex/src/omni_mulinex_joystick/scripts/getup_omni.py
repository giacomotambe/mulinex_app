#!/usr/bin/env python3

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from pi3hat_moteus_int_msgs.msg import JointsCommand, JointsStates, PacketPass
from sensor_msgs.msg import  Imu, JointState
from geometry_msgs.msg import Twist 


""" get up the robot by sending refernces interpolating from current to default pos """

class GetUp(Node):
    def __init__(self):
        super().__init__('getup_traq')

        # Topic names
        self.declare_parameter('joint_target_pos_topic', '/joint_controller/command')
        self.joint_target_pos_topic = self.get_parameter('joint_target_pos_topic').get_parameter_value().string_value

        self.simulation = False

        self.deltaT = 5.0
        self.rate = 100
        self.default_dof = np.array([
            2.2,    
            -1.3,   
            -2.2,    
            1.3,
            -2.2,    
            1.3,
            2.2,    
            -1.3,                   
            np.nan,
            np.nan,
            np.nan,
            np.nan
        ]) 

        # Initialize joint publisher/subscriber
        self.njoint = 12

        self.joint_names=(
            'LF_HFE',
            'LF_KFE',   # flip
            'LH_HFE',
            'LH_KFE',   # flip
            'RF_HFE',
            'RF_KFE',
            'RH_HFE',
            'RH_KFE',
            'LF_WHEEL', 
            'LH_WHEEL',
            'RF_WHEEL',
            'RH_WHEEL',
        )

        self.default_vel = np.array([
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]) 
        self. i = 0
        self.init_vel = {self.joint_names[i]:0.0 for i in range(self.njoint)}
        self.init_pos = {self.joint_names[i]:0.0 for i in range(self.njoint)}
        
        self.declare_parameter('joint_state_topic', '/state_broadcaster/joints_state')
        self.joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value

        self.current_pos = np.zeros(self.njoint)
        self.last_valid_pos = np.zeros(self.njoint)

        self.valid_state_acquired = False
        self.nan_counter = 0
        self.nan_limit = 20  # cicli consecutivi prima di SAFE

        self.last_msg_time = self.get_clock().now()
        self.state_timeout = 0.5  # secondi

        self.max_position_jump = 1.0  # rad, anti-glitch
        self.safe_mode = False
        self.joint_target_pos_pub = self.create_publisher(JointsCommand, self.joint_target_pos_topic, 10)
        self.joint_sub  = self.create_subscription(JointsStates, self.joint_state_topic, self.joint_state_callback, 10)
        
        self.default_acquired = False

        rclpy.logging.get_logger('rclpy.node').info('GetUp started, waiting for joint state') 


    def getup_callback(self):
        WARMUP_ZONE = 100
        start_pos = np.array(list(self.init_pos.values()))
        start_vel = np.array(list(self.init_vel.values()))
        
        if self.i < WARMUP_ZONE:
            self.joint_pos =  start_pos
            self.joint_vel = start_vel

        else:
            # interpolate from current to default pos
            t = ((self.i - WARMUP_ZONE)/ (self.deltaT * self.rate))
            

            # linear interpolation:
            self.joint_pos = start_pos*(1-t) + self.default_dof*t
            # self.joint_vel =  self.default_vel
            self.joint_vel = start_vel



        rclpy.logging.get_logger('rclpy.node').info(f'joint pos: {self.joint_pos}') 
        rclpy.logging.get_logger('rclpy.node').info(f'i: {self.i}') 

        self.i += 1
        
        # SATURATE WARMPU_ZONE
        if self.i > WARMUP_ZONE + self.deltaT * self.rate:
            self.i = WARMUP_ZONE +  self.deltaT * self.rate

            self.joint_pos = self.default_dof
            self.joint_vel = start_vel

        joint_msg = JointsCommand()


        joint_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = (self.joint_pos).tolist()
        joint_msg.velocity = (self.joint_vel).tolist()
        joint_msg.effort = np.zeros(self.njoint).tolist()
        # if not self.simulation:
        joint_msg.kp_scale = np.ones(self.njoint).tolist()
        joint_msg.kd_scale = np.ones(self.njoint).tolist()

        self.joint_target_pos_pub.publish(joint_msg)


    def joint_state_callback(self, msg):

        now = self.get_clock().now()
        positions = np.array(msg.position)

        # ----------------------------------------------------------
        # START CONDITION: require zero NaN before starting the node
        # ----------------------------------------------------------
        if not self.valid_state_acquired:
            if np.any(np.isnan(positions)):
                self.get_logger().warn("Waiting for joint states with ZERO NaN...")
                return

            # If you want to require only legs (0:8) to be valid, use:
            # if np.any(np.isnan(positions[0:8])): return

        valid_measurement = False

        for i in range(self.njoint):

            if not np.isnan(positions[i]):

                # Anti-glitch: block huge jumps
                if self.valid_state_acquired:
                    if abs(positions[i] - self.last_valid_pos[i]) > self.max_position_jump:
                        self.get_logger().warn(f"Jump detected on joint {i}, ignoring")
                        continue

                self.current_pos[i] = positions[i]
                self.last_valid_pos[i] = positions[i]
                valid_measurement = True

        if valid_measurement:
            self.nan_counter = 0
            self.last_msg_time = now

            if not self.valid_state_acquired:
                # Also initialize init_pos from the first fully-valid state
                for i in range(self.njoint):
                    self.init_pos[self.joint_names[i]] = float(self.last_valid_pos[i])
                    self.init_vel[self.joint_names[i]] = 0.0

                self.valid_state_acquired = True
                self.timer = self.create_timer(1.0/self.rate, self.getup_callback)  # start getup
                self.get_logger().info("Valid joint state received (ZERO NaN). GetUp started.")

        else:
            self.nan_counter += 1
            if self.nan_counter > self.nan_limit:
                self.safe_mode = True
                self.get_logger().error("Too many NaN states → SAFE MODE")



def main(args=None):
    rclpy.init(args=args)
    getup = GetUp()
    rclpy.spin(getup)
    getup.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
