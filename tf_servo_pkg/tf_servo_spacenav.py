#!/usr/bin/env python3

'''
FILE:   tf_servo_spacenav.py
DESC:   Basic spatial velocity controller using a spacenav device
DATE:   June 18 2021
AUTH:   scott@restfulrobotics.com

NOTE

To install the space navigator dependencies

$ git clone https://github.com/mastersign/pyspacenav.git
$ sudo apt-get install spacenavd libspnav-dev
$ python3 setup.py build
$ sudo python3 setup.py install
$ python3 example.py

Topics
    /delta_twist_cmds, published TwistStamped generated from the spacemouse input


'''

import sys

import spacenav

import numpy as np
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class TF_Servo_SpaceNav( Node ):

    def __init__(self, 
        scale_translation=np.array( [   [0.01, 0.0, 0.0],
                                        [0.0, 0.0, 0.01],
                                        [0.0, 0.01, 0.0] ] ),
        scale_rotation=np.array( [  [0.02, 0.0, 0.0],
                                    [0.0, 0.0, 0.02],
                                    [0.0, 0.02, 0.0]] ) ):

        super().__init__('tf_servo_spacenav')


        self.scale_translation = scale_translation
        self.scale_rotation = scale_rotation

        #Try to open a connection to device
        try:
            spacenav.open()
        except spacenav.ConnectionError:
            self.get_logger().error(f'Unable to open SpaceNav device, exiting....')
            sys.exit()
        else:
            if spacenav.is_connected:
                self.get_logger().info('Opened SpaceNav device...')
            else:
                self.get_logger().error(f'Unable to open SpaceNav device, exiting....')
                sys.exit()

        self.twist_pub = self.create_publisher( TwistStamped, 'delta_twist_cmds', 10)

        # create a timer to poll for spacemouse events
        self.timer = self.create_timer( 1.0 / 25.0, self.spacenav_callback )

    def spacenav_callback( self ):

        translation = np.array([[0.0], [0.0], [0.0]])
        rotation = np.array([[0.0], [0.0], [0.0]])

        try:
            event = spacenav.poll()

            if event:

                #clear event queues to prevent accumulation
                spacenav.remove_events(spacenav.EVENT_TYPE_MOTION)
                spacenav.remove_events(spacenav.EVENT_TYPE_BUTTON)

                if type(event) == spacenav.MotionEvent:

                    dxdt = np.array( [[float(event.x)/350.0], [float(event.y)/350.0], [float(event.z)/350.0]] )
                    drdt = np.array( [[float(event.rx)/350.0], [float(event.ry)/350.0], [float(event.rz)/350.0]] )

                    translation = self.scale_translation @ dxdt
                    rotation = self.scale_rotation @ drdt

        except spacenav.ConnectionError:
            pass
        finally:
            msg_twist = TwistStamped()
            msg_twist.header.frame_id = 'world'
            msg_twist.header.stamp = self.get_clock().now().to_msg()
            msg_twist.twist.linear.x = translation[0,0]
            msg_twist.twist.linear.y = translation[1,0]
            msg_twist.twist.linear.z = translation[2,0]
            msg_twist.twist.angular.x = rotation[0,0]
            msg_twist.twist.angular.y = rotation[1,0]
            msg_twist.twist.angular.z = rotation[2,0]

            self.twist_pub.publish(msg_twist)    


def main(args=None):
    rclpy.init(args=args)
    tf_spacenav_node = TF_Servo_SpaceNav()
    tf_spacenav_node.get_logger().info('Created tf_spacenav node...')
    rclpy.spin(tf_spacenav_node)
    tf_spacenav_node.destroy_node()
    spacenav.close()
    rclpy.shutdown()
    pass

if __name__=='__main__':
    main()





    




