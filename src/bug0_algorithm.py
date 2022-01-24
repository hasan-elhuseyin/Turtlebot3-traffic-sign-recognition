#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt
import random
from sensor_msgs.msg import LaserScan
import time


class AvoidObstacles:

    def __init__(self):
        # set the goal and robot initial info
        self.xr = 0
        self.yr = 9

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.disthr = 0.1
        self.KpL = 0.7
        self.KpA = 3.0

        self.max_vel = 0.8
        self.set_vel = Twist()

        self.max_distance = 1.1 # max distance to obstacles

        self.regions = {
            'front1':  10,
            'front2':  10,
            'right':  10,
            'left':  10
        }
        self.turnSign = ''
        self.recognition = False


    def bug0_algorithm(self):

        self.sub_odometry = rospy.Subscriber('odom/', Odometry,  self.callback_odometry_msg)
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.callback_laser)
        self.sub_classification = rospy.Subscriber('/classification_result', String, self.callback_classification_result)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        self.obstacle_exists = False

        while self.get_euclidean_distance() > self.disthr:
            if self.obstacle_exists is False:
                self.go_to_goal()
            else:
                if self.turnSign == '':
                    self.recognition = True
                while(self.turnSign == ''):
                    print("Waiting vision node to initialize..")
                    time.sleep(5)
                # check direction
                if(self.turnSign == "left"):
                    self.follow_left_wall()
                    #print('left is working')
                    self.recognition = False
                elif(self.turnSign == "right"):
                    #print('right is working')
                    self.follow_right_wall()
                    self.recognition = False
        self.turnSign = ''


    def go_to_goal(self):
        self.turnSign = ''
        print('Go to goal')

        # go to goal while no obstacle exists in the front of the robot
        while self.get_euclidean_distance() > self.disthr and self.obstacle_exists is False:

            # if any obstacle exists, set obstacle_exists to True and call follow_wall function
            if self.regions['front1'] < self.max_distance and self.regions['front2'] < self.max_distance:
                # #------ todo: delete this section and use image recognition data instead ------#
                # # self.direction = input("Enetr direction (1. right 2. left): ")
                # self.direction = random.randint(0, 1)
                # directionArr = ['right', 'left']
                # print('direction: ' + directionArr[self.direction])
                # #------ todo: delete this section and use image recognation data ------#
                self.obstacle_exists = True

            # go to the goal
            self.set_vel.linear.x = self.KpL * min(self.get_euclidean_distance(), self.max_vel)
            self.set_vel.angular.z = self.KpA * (atan2(self.yr - self.y, self.xr - self.x) - self.theta)
            self.pub_vel.publish(self.set_vel)
            self.rate.sleep()

        # stop when the robot arrives the goal
        self.set_vel.linear.x = 0.0
        self.set_vel.angular.z = 0.0
        print("Reached the goal!")
        self.pub_vel.publish(self.set_vel)

    # follow wall function
    def follow_left_wall(self):
        if self.regions['front1'] < self.max_distance and self.regions['front2'] < self.max_distance:
            self.turn_left()
        elif self.regions['front1'] < self.max_distance and self.regions['front2'] < self.max_distance and self.regions['right'] < self.max_distance:
            self.turn_left()
        elif self.regions['front1'] < self.max_distance and self.regions['front2'] < self.max_distance and self.regions['left'] < self.max_distance:
            self.turn_left()
        elif self.regions['front1'] > self.max_distance and self.regions['front2'] > self.max_distance and self.regions['left'] > self.max_distance and self.regions['right'] < self.max_distance:
            self.follow_the_wall()

        if self.regions['front1'] > self.max_distance and self.regions['front2'] > self.max_distance and self.regions['left'] > self.max_distance and self.regions['right'] > self.max_distance:
            self.obstacle_exists = False

        self.pub_vel.publish(self.set_vel)

    # follow wall function
    def follow_right_wall(self):
        if self.regions['front1'] < self.max_distance and self.regions['front2'] < self.max_distance:
            self.turn_right()
        elif self.regions['front1'] < self.max_distance and self.regions['front2'] < self.max_distance and self.regions['left'] < self.max_distance:
            self.turn_right()
        elif self.regions['front1'] < self.max_distance and self.regions['front2'] < self.max_distance and self.regions['right'] < self.max_distance:
            self.turn_right()
        elif self.regions['front1'] > self.max_distance and self.regions['front2'] > self.max_distance and self.regions['right'] > self.max_distance and self.regions['left'] < self.max_distance:
            self.follow_the_wall()

        if self.regions['front1'] > self.max_distance and self.regions['front2'] > self.max_distance and self.regions['right'] > self.max_distance and self.regions['left'] > self.max_distance:
            self.obstacle_exists = False

        self.pub_vel.publish(self.set_vel)


    # turn right function
    def turn_right(self):
        # print('turn right')
        self.set_vel.linear.x = 0
        self.set_vel.angular.z = -0.3

    # turn left function
    def turn_left(self):
        # print('turn left')
        self.set_vel.linear.x = 0
        self.set_vel.angular.z = 0.3

    # follow wall function
    def follow_the_wall(self):
        # print('follow the wall')
        self.set_vel.linear.x = 0.3
        self.set_vel.angular.z = 0

    # get euclidean distance function
    def get_euclidean_distance(self):
        return sqrt(pow((self.x - self.xr), 2) + pow((self.y - self.yr), 2))

    # get robot position
    def callback_odometry_msg(self, data):
        qtrn = data.pose.pose.orientation
        [roll, pitch, self.theta] = euler_from_quaternion(
            [qtrn.x, qtrn.y, qtrn.z, qtrn.w])

        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

    # get laser sensor regions messages
    def callback_laser(self, msg):
        self.regions = {
            'front1':  min(min(msg.ranges[340:359]), 10), # [330:359]
            'front2':  min(min(msg.ranges[0:29]), 10), # [0:29]
            'left':  min(min(msg.ranges[30:90]), 10),
            'right':  min(min(msg.ranges[260:329]), 10) # [260:329]
        }
    
    def callback_classification_result(self, msg):
        if self.recognition == True:
            print("detected sign is: " + msg.data)
            self.turnSign = msg.data
            self.recognition = False

if __name__ == '__main__':
    rospy.init_node("robot_control", anonymous=True)

    time.sleep(3)

    x = AvoidObstacles()
    x.bug0_algorithm()

    rospy.spin()