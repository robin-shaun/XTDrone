#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

import math
import numpy
import sys
import termios

instructions = """
Reading from the keyboard and Publishing Thrust Angles!
---------------------------
Change Thrust Angle clockwise: h
Change Thrust Angle counter-clockwise: ;

r/v : increase/decrease thruster angle change speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'h': -1,
        ';': 1,
    }

speedBindings = {
        'r': 1,
        'v': -1,
    }


# Implement getch function, which reads in 1 char from user input.
# Purposely different from teleop_twist_keyboard.py b/c of
# issues outputting to stdout when they both read from the input.
# Reference https://gist.github.com/houtianze/9e623a90bb836aedadc3abea54cf6747

def __gen_ch_getter(echo):

    def __fun():
        fd = sys.stdin.fileno()
        oldattr = termios.tcgetattr(fd)
        newattr = oldattr[:]
        try:
            if echo:
                # disable ctrl character printing, otherwise,
                # backspace will be printed as "^?"
                lflag = ~(termios.ICANON | termios.ECHOCTL)
            else:
                lflag = ~(termios.ICANON | termios.ECHO)
            newattr[3] &= lflag
            termios.tcsetattr(fd, termios.TCSADRAIN, newattr)
            ch = sys.stdin.read(1)
            if echo and ord(ch) == 127:  # backspace
                # emulate backspace erasing
                # https://stackoverflow.com/a/47962872/404271
                sys.stdout.write('\b \b')
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, oldattr)
        return ch
    return __fun


if __name__ == "__main__":
    # Setup getch function
    getch = __gen_ch_getter(False)

    # Setup ros publishers and node
    left_pub = rospy.Publisher('left_thrust_angle', Float32, queue_size=1)
    right_pub = rospy.Publisher('right_thrust_angle', Float32, queue_size=1)
    rospy.init_node('key2thrust_angle')

    # Initialize current angle and angle speed
    thrust_angle_speed = 0.1
    max_angle = rospy.get_param("~max_angle", math.pi / 2)
    curr_angle = 0
    num_prints = 0

    try:
        # Output instructions
        print(instructions)
        print('Max Angle: {}'.format(max_angle))
        while(1):
            # Read in pressed key
            key = getch()

            if key in moveBindings.keys():
                # Increment angle, but clip it between [-max_angle, max_angle]
                curr_angle += thrust_angle_speed * moveBindings[key]
                curr_angle = numpy.clip(curr_angle,
                                        -max_angle, max_angle).item()

            elif key in speedBindings.keys():
                # Increment/decrement speed of angle change and print it
                thrust_angle_speed += (speedBindings[key] * 0.1 *
                                       thrust_angle_speed)
                print('currently:\t'
                      'thruster angle speed {} '.format(thrust_angle_speed))

                # Reprint instructions after 14 speed updates
                if (num_prints == 14):
                    print(instructions)
                num_prints = (num_prints + 1) % 15

            elif key == '\x03':
                break

            # Publish thrust angle
            angle_msg = Float32()
            angle_msg.data = curr_angle
            left_pub.publish(angle_msg)
            right_pub.publish(angle_msg)

    except Exception as e:
        print(e)

    finally:
        # Send 0 angle command at end
        angle_msg = Float32()
        angle_msg.data = 0
        left_pub.publish(angle_msg)
        right_pub.publish(angle_msg)
