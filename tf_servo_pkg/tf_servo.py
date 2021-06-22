#!/usr/bin/env python3
'''
FILE:   tf_servo.py
DESC:   Simple application to drive a coordinate frame via moveit2_servo_teleop
DATE:   June 18 2021
AUTH:   scott@restfulrobotics.com

EXAMPLE USEAGE


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

    def __init__(self):
        super().__init__('tf_servo')

        self.pose_pub = self.create_publisher( PoseStamped, 'tf_servo', 10 )

        self.twist_sub = self.create_subscription( TwistStamped, 'delta_twist_cmds', self.twist_callback, 10 )

        self.tf_servo = np.identity(4)


    def twist_callback( self, twist ):
        #self.get_logger().info('twist received...')

        #[[0,-c,b],[c,0,-a],[-b,a,0]],
        S = np.array( [
            [0, -twist.twist.angular.z, twist.twist.angular.y],
            [twist.twist.angular.z, 0, -twist.twist.angular.x],
            [-twist.twist.angular.y, twist.twist.angular.x, 0] 
        ] )

        dR = S @ self.tf_servo[0:3,0:3]
        dx = np.array( [twist.twist.linear.x,twist.twist.linear.y, twist.twist.linear.z] )

        # Use SVD to preserve SE3 properties as dR = S(w)*R is an approximation
        U,_,VH = LA.svd(dR + self.tf_servo[0:3, 0:3]) # note VH = V.T
        RR = U @ VH
        self.tf_servo[0:3, 0:3] = RR
        self.tf_servo[0:3,3] = dx + self.tf_servo[0:3,3] 

        tx=PoseStamped()
        tx.header.frame_id = 'base_link'
        tx.header.stamp = self.get_clock().now().to_msg()
        tx.pose.position.x = self.tf_servo[0,3]
        tx.pose.position.y = self.tf_servo[1,3]
        tx.pose.position.z = self.tf_servo[2,3]
        quat = Quaternion(matrix=self.tf_servo[0:3,0:3])
        tx.pose.orientation.w = quat.elements[0]
        tx.pose.orientation.x = quat.elements[1]
        tx.pose.orientation.y = quat.elements[2]
        tx.pose.orientation.z = quat.elements[3]

        self.pose_pub.publish(tx)



def main(args=None):
    rclpy.init(args=args)
    tf_servo_node = TF_Servo()
    tf_servo_node.get_logger().info('Created tf_servo_node...')
    rclpy.spin(tf_servo_node)
    tf_servo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
