#!/usr/bin/env python3
'''
FILE:   tf_servo.py
DESC:   Simple application to drive a coordinate frame via moveit2_servo_teleop
DATE:   June 18 2021
AUTH:   scott@restfulrobotics.com
NOTE:

This node uses the following topics

1) [published] /tf_servo
Publishes a time integrated transform; displacements are accumulated via
'delta_twist_commands', message type PoseStamped

2) [subscribed] /delta_twist_cmds
Subscribes to messages of type TwistStamped to affect the transform
tf_servo

3) [subscribed] /tf_reset
Subscribes to message of type PoseStamped; if received, overwrites the tf_servo transform

Example to publish from the command line to reset the tf_servo value

ros2 topic pub --once /tf_reset geometry_msgs/PoseStamped \
    '{header: {frame_id: "base_link"}, pose: {position: {x: 0, y: 0, z: 0}, orientation: {w: 1.0}}}'



'''

import numpy as np
from numpy import linalg as LA
from pyquaternion import Quaternion

import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from tf2_ros import TransformStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped


class TF_Servo( Node ):

    def __init__(self, verbose=False):
        super().__init__('tf_servo')

        self.verbose = verbose

        self.tx_servo_pub = self.create_publisher( PoseStamped, 'tf_servo', 10 )
        self.twist_sub = self.create_subscription( TwistStamped, 'delta_twist_cmds', self.twist_callback, 10 )
        self.tf_reset_sub = self.create_subscription( PoseStamped, 'tf_reset', self.tf_reset_callback, 10 )
        self.tx_servo = np.identity(4)

        self.tf_reset = None

    def tf_reset_callback( self, tf_reset ):        
        
        if  None == self.tf_reset:
            self.tf_reset = tf_reset
            
            

    def twist_callback( self, twist ):

        if self.tf_reset:
            # Convert tf into 4x4 transform
            self.tx_servo[0,3] = self.tf_reset.pose.position.x
            self.tx_servo[1,3] = self.tf_reset.pose.position.y
            self.tx_servo[2,3] = self.tf_reset.pose.position.z
            #conv quaternion into 3x3
            #http://kieranwynn.github.io/pyquaternion/
            qq = Quaternion( w=self.tf_reset.pose.orientation.w, x=self.tf_reset.pose.orientation.x, \
                y=self.tf_reset.pose.orientation.y, z=self.tf_reset.pose.orientation.z )
            self.tx_servo[0:3,0:3] = qq.rotation_matrix
            self.tf_reset = None
            if self.verbose:
                self.get_logger().info('Reset tf_servo...\n')

        # Given twist vector, convert to skew symmetric matrix
        #[[0,-c,b],[c,0,-a],[-b,a,0]],
        S = np.array( [
            [0, -twist.twist.angular.z, twist.twist.angular.y],
            [twist.twist.angular.z, 0, -twist.twist.angular.x],
            [-twist.twist.angular.y, twist.twist.angular.x, 0] 
        ] )

        # Get a differential rotation matrix; dR/dt = S(w)*R is an approximation
        dR = S @ self.tx_servo[0:3,0:3]
        dx = np.array( [twist.twist.linear.x,twist.twist.linear.y, twist.twist.linear.z] )

        # Use SVD to preserve SO3 properties
        U,_,VH = LA.svd(dR + self.tx_servo[0:3, 0:3]) # note VH = V.T
        RR = U @ VH
        self.tx_servo[0:3, 0:3] = RR
        self.tx_servo[0:3,3] = dx + self.tx_servo[0:3,3] 

        tfps=PoseStamped()
        tfps.header.frame_id = 'world'
        tfps.header.stamp = self.get_clock().now().to_msg()
        tfps.pose.position.x = self.tx_servo[0,3]
        tfps.pose.position.y = self.tx_servo[1,3]
        tfps.pose.position.z = self.tx_servo[2,3]
        quat = Quaternion(matrix=self.tx_servo[0:3,0:3])
        tfps.pose.orientation.w = quat.elements[0]
        tfps.pose.orientation.x = quat.elements[1]
        tfps.pose.orientation.y = quat.elements[2]
        tfps.pose.orientation.z = quat.elements[3]

        self.tx_servo_pub.publish( tfps )

def main(args=None):
    rclpy.init(args=args)
    tf_servo_node = TF_Servo()
    tf_servo_node.get_logger().info('Created tf_servo_node...')
    rclpy.spin(tf_servo_node)
    tf_servo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
