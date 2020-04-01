#!/usr/bin/env python
"""
Wall Follower ros package script
Author: Zach Mills
"""

import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from itertools import product
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelStates
import random
import numpy as np
import pickle
import os

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
        self.model_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.check_model)
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

        self.rate = rospy.Rate(10) # 10Hz

        self.consecutive_stuck = 0
        self.episodes = 0
        self.need_reset = False
        self.point = None
        self.same_state_thresh = 0.01
        self.consecutive_thresh = 10

        self.state = (2, 2, 2)
        self.current_subs = [2, 2, 2]

        self.previous_state = (2, 2, 2)
        self.action_taken = 1

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

        # Hyperparameters
        self.epsilon = 0.3
        self.initial_epsilon = 0.3
        self.epsilon_decay = 0.99
        self.alpha = 0.8
        self.gamma = 0.5

        # Define values for operations
        dodge = 5
        stay = 1
        approach = 2
        avoid = 3

        self.q_table = {(s, a): 0 for a in actions for s in states}
        for pair in self.q_table.keys():
            if pair[0][2] == 1 and pair[1] == 1:
                self.q_table[pair] = stay
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

    def check_model(self, data):
        """Callback function for Model State"""

        point = data.pose[1].position

        if point.z > 0.1:
            self.need_reset = True
            return

        point = np.array([point.x, point.y, point.z])
        if self.point is None:
            self.point = point
            return

        if np.allclose(self.point, point, self.same_state_thresh, 0):
            self.consecutive_stuck += 1

        if self.consecutive_stuck > self.consecutive_thresh:
            self.need_reset = True

        self.point = point



    def calculate_reward(self):
        reward = 0
        if 0 in self.state:
            reward = -1
        if self.state[2] == 2:
            reward = -1

        return reward

    def get_pose(self, action):
        pose =  Pose2D()
        pose.x, pose.y, pose.theta = self.pose_dict[action]

        return pose

    def take_action(self, action):
        # print "taking action", action
        self.previous_state = self.state
        self.action_taken = action

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

    def train_follow(self):
        while not rospy.is_shutdown():
            next_action = 1
            max_value = 0

            self.pause_physics

            for pair in self.q_table.keys():
                if pair[0] == self.state and self.q_table[pair] > max_value:
                    next_action = pair[1]
                    max_value = self.q_table[pair]

            next_action = self.choose_action()

            self.take_action(next_action)
            self.rate.sleep()
            self.update_q(self.calculate_reward())

            self.unpause_physics

            if self.episodes > 10000:
                return

            if self.need_reset:
                if self.episodes % 100 == 0:
                    print self.episodes
                with open(os.path.abspath(os.path.dirname( __file__ ))
                          + '/q_table.pickle', 'wb') as file:
                    pickle.dump(follower.q_table, file)
                self.episodes += 1
                self.epsilon = self.initial_epsilon * (1 -(self.epsilon_decay ** episode))
                self.reset_world()
                self.need_reset = False
                self.consecutive_stuck = 0

    def choose_action(self):
        options = {}

        for pair in self.q_table.keys():
            if pair[0] == self.state:
                options[pair] = self.q_table[pair]

        if random.random() < self.epsilon:
            next_action = max(options, key=options.get)[1]
        else:
            next_action = random.choice(options.keys())[1]

        return next_action

    def update_q(self, reward=0):
        current = self.q_table[(self.previous_state, self.action_taken)]
        best = max([pair for pair in self.q_table if pair[0] == self.state],
                   key=self.q_table.get)

        updated = current + self.alpha * (reward + (self.gamma * self.q_table[best]) - current)

        self.q_table[(self.previous_state, self.action_taken)] = updated


if __name__ == "__main__":
    follower = Follower()

    choice = None
    while choice not in [1, 2]:
        try:
            choice = int(raw_input("Train (1) or Follow (2): "))
        except TypeError:
            continue

    if choice == 1:
        follower.train_follow()
    else:
        try:
            with open(os.path.abspath(os.path.dirname( __file__ ))
                      + '/q_table.pickle', 'rb') as file:
                follower.q_table = pickle.load(file)
            follower.follow()
        except IOError:
            print "Q Table not found, try training first."
