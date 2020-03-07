#!/usr/bin/env python

import os
import rospkg
import math
import rospy
from geometry_msgs.msg import Twist

def draw_m(steps):
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('draw_m', anonymous=True)
    rate = rospy.Rate(1) # 1 Hz

    count = 0

    while not rospy.is_shutdown():
        rospy.loginfo("driving")
        vel = steps[count]
        pub.publish(vel)
        rate.sleep()
        count = (count + 1) % len(steps)


def read_coords(filename):
    '''Read list of coordinates from a filename and output a list of tuples
    '''

    return [c.strip().split(',') for c in open(filename, 'r').readlines()]


def parse_coords(coords):
    '''Take a list of coordinate pairs and output a list of Twists'''

    coords = list(map(lambda c: (float(c[0]), float(c[1])), coords))

    count = 0
    previous_loc = coords[-1]  # Take the last coordinate as the first previous
    previous_angle = 0

    output = []

    while count < len(coords):
        next_loc = coords[count]
        new_angle = math.atan2(next_loc[1] - previous_loc[1],
                               next_loc[0] - previous_loc[0])
        rotation = Twist()
        rotation.angular.z = previous_angle - new_angle
        distance = math.sqrt(((next_loc[1] - previous_loc[1])**2
                              + (next_loc[0] - previous_loc[0])**2))
        translation = Twist()
        translation.linear.x = distance
        output.append(rotation)
        output.append(Twist())  # Spacer
        output.append(translation)

        previous_loc = next_loc
        previous_angle = new_angle

        count += 1

    # Need to reset our rotation to 0 radians

    new_angle = 0
    rotation = Twist()
    rotation.angular.z = previous_angle - new_angle
    output.append(rotation)

    return output



if __name__ == '__main__':
    try:
        rp = rospkg.RosPack()
        data_path = os.path.join(rp.get_path("mines_mills_zach"),
                                 "data", "coords.txt")
        coords = read_coords(file)
        steps = parse_coords(coords)
        draw_m(steps)
    except rospy.ROSInterruptException:
        pass
