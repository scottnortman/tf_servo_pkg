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

from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from rclpy.exceptions import ParameterNotDeclaredException, ParameterAlreadyDeclaredException, InvalidParameterException, InvalidParameterValueException

from rclpy.logging import get_logger
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class TF_Servo_SpaceNav( Node ):

    def __init__( self ):

        super().__init__('tf_servo_spacenav')

        scale_translation_default = '0.01, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.01, 0.0'
        scale_rotation_default = '0.01, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.01, 0.0'

        # Declare gain matricies as parameters to enable launch file update
        scale_translation_descriptor = ParameterDescriptor( description='3x3 Translation Scale Matrix as a 9 element string, row major format.' )
        try:
            self.declare_parameter( 'scale_translation', scale_translation_default, scale_translation_descriptor)
        except ParameterAlreadyDeclaredException as e:
            self.get_logger().warn(f'Parameter Exception: [{e}]')
        except InvalidParameterException as e:
            self.get_logger().warn(f'Parameter Exception: [{e}]')
        except InvalidParameterValueException as e:
            self.get_logger().warn(f'Parameter Exception: [{e}]')
        
        scale_rotation_descriptor = ParameterDescriptor( description='3x3 Rotation Scale Matrix as a 9 element string, row major format.')
        try:
            self.declare_parameter( 'scale_rotation', scale_rotation_default, scale_rotation_descriptor )
        except ParameterAlreadyDeclaredException as e:
            self.get_logger().warn(f'Parameter Exception: [{e}]')
        except InvalidParameterException as e:
            self.get_logger().warn(f'Parameter Exception: [{e}]')
        except InvalidParameterValueException as e:
            self.get_logger().warn(f'Parameter Exception: [{e}]')
        
        # Get parameters
        try:
            scale_translation = self.get_parameter( 'scale_translation' ).get_parameter_value().string_value
        except ParameterNotDeclaredException as e:
            self.get_logger().warn(f'Parameter Exception: [{e}]')
            scale_translation = scale_translation_default
        finally:
            self.scale_translation = self.parameter_string_to_3x3( scale_translation )

        self.get_logger().info(f'\nscale_translation:\n[{self.scale_translation}]\n')

        try:
            scale_rotation = self.get_parameter( 'scale_rotation' ).get_parameter_value().string_value
        except ParameterNotDeclaredException as e:
            self.get_logger().warn(f'Parameter Exception: [{e}]')
            scale_rotation = scale_rotation_default
        finally:
            self.scale_rotation = self.parameter_string_to_3x3( scale_rotation )

        self.get_logger().info(f'\nscale_rotation:\n[{self.scale_rotation}]\n')


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

    def parameter_string_to_3x3( self, param_string ):
        '''
        Converts a comma delimited string into a 3x3 np matrix array

        '''
        
        matrix_3x3 = np.zeros([3,3])

        # Break string with ',' delimiter, ignore whitespace
        ss = param_string.split(',')

        idx=0
        for r in range(3):
            for c in range(3):
                matrix_3x3[r,c] = float(ss[idx])
                idx += 1

        return matrix_3x3

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





    




