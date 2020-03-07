#!/usr/bin/env python
"""
Wall Follower ros package script
Author: Zach Mills
"""

import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from itertools import product

class Follower:
    """
    Defines Publishers and Subscribers for wall-following
    """

    def __init__(self):
        """Construct a Follower class"""

        rospy.init_node('Wall_Follower', anonymous=True)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.get_dist)
        self.pub = rospy.Publisher('/triton_lidar/vel_cmd',
                                   Pose2D, queue_size=10)

        self.rate = rospy.Rate(10) # 10Hz


        self.state = (2, 2, 2)
        self.current_subs = [2, 2, 2]

        # Set thresholds
        self.thresh = {0: [0.5, 1.0],
                       1: [0.5, 1.5],
                       2: [0.25, 0.5]}

        # Set poses
        self.pose_dict = {0: (0, 2, 5),
                          1: (0, 2, 0),
                          2: (0, 2, -5)}

        # Initialize Q-table
        substates = [0, 1, 2]  # close, mid, far
        actions = [0, 1, 2]  # left, straight, right
        states = [s for s in product(substates, repeat=3)]

        # Define values for operations
        dodge = 5
        stay = 1
        approach = 2
        avoid = 3

        self.q_table = {(s, a): 0 for a in actions for s in states}
        for pair in self.q_table.keys():
            if pair[0][1] < 2 and pair[1] == 0:
                self.q_table[pair] = dodge
            if pair[0][2] == 1 and pair[1] == 1:
                self.q_table[pair] = stay
            if pair[0][2] == 2 and pair[1] == 2:
                self.q_table[pair] = approach
            if pair[0][2] == 0 and pair[1] == 0:
                self.q_table[pair] = avoid
            if pair[0][0] == 0 and pair[1] == 2:
                self.q_table[pair] = avoid
            if pair[0][0] == 2 and pair[0][1] == 2 and pair[0][2] == 2:
                self.q_table[pair] = stay

    def get_dist(self, data):
        """Callback function for LIDAR data"""

        right = data.ranges[0]
        front = data.ranges[89]
        left = data.ranges[179]

        dists = [left, front, right]
        self.current_subs = [2, 2, 2]

        for t, source in enumerate(dists):
            for bound in range(len(self.thresh[0])):
                if source <= self.thresh[t][bound]:
                    self.current_subs[t] = bound
                    break

        self.state = tuple(self.current_subs)

    def get_pose(self, action):
        pose =  Pose2D()
        pose.x, pose.y, pose.theta = self.pose_dict[action]

        return pose

    def take_action(self, action):
        print "taking action", action
        self.pub.publish(self.get_pose(action))

    def follow(self):
        while not rospy.is_shutdown():
            print self.current_subs
            next_action = 1
            max_value = 0

            for pair in self.q_table.keys():
                if pair[0] == self.state and self.q_table[pair] > max_value:
                    next_action = pair[1]
                    max_value = self.q_table[pair]

            self.take_action(next_action)
            self.rate.sleep()

if __name__ == "__main__":
    follower = Follower()
    follower.follow()
