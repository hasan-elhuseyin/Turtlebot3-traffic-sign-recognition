#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, atan2


class RobotControl:

    def __init__(self):
        # vaiables.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.KpL = 0.7
        self.KpA = 3.0
        self.max_vel = 1.0
        self.set_vel = Twist()
        # minimum distance to goal and obstacle.
        self.distanceG = 0.05 # distance to Goal
        self.distanceO =  0.0 # distance to Obstacle
        self.obstacleInFront = False
        # pubs and subs.
        self.sub = rospy.Subscriber('/odom', Odometry,  self.callback_odometry_msg)
        self.scan = rospy.Subscriber('/scan', LaserScan, self.callback_laser)
        #self.classificationSub = rospy.Subscriber('/classification_result', String, self.callback_classification_result)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        # angle regions for LaserScan.
        global regions
        regions = {
            'front': 10,
            'right': 10,
            'left': 10 }


    def hedefeGit(self, xr, yr):
        while self.get_euclidean_distance(xr, yr) > self.distanceG and self.obstacleInFront == False:
            # if there is an obstacle in the way change the status to wall following
            if regions['front'] < self.distanceO:
                self.obstacleInFront = True

            self.set_vel.linear.x = self.KpL * min(self.get_euclidean_distance(xr, yr), self.max_vel)
            self.set_vel.angular.z = self.KpA * (atan2(yr - self.y, xr - self.x) - self.theta)
            self.pub.publish(self.set_vel)
            self.rate.sleep()
            # print("distance to goal: " + str(self.get_euclidean_distance(xr, yr)))

            while self.obstacleInFront:
                self.checkObstaclesAndMoveAccordingly()

        self.set_vel.linear.x = 0.0
        self.set_vel.angular.z = 0.0
        self.pub.publish(self.set_vel)
        #print("Reached the point: " + str(xr) + ", " + str(yr))


    # Run this function inside while loop
    def checkObstaclesAndMoveAccordingly(self):
        # if the obstacle is on the front, check sign direction
        if regions['front'] < self.distanceO:
            if('right' == 'right'):
                self.followRightWall(self)
            elif(self.getDirection(self) == 'left'):
                self.followLeftWall(self)

    def followRightWall(self):
        # rotate to the left
        if regions['front'] < self.distanceO:
            self.turnLeft()
        # if the obstacle is on the right, follow the wall
        elif regions['front'] > self.distanceO and regions['right'] < self.distanceO:
            self.followTheWall()

        # if there is no obstacles on front or right, end while loop
        if regions['front'] > self.distanceO and regions['right'] > self.distanceO:
            self.obstacleInFront = False

        self.pub.publish(self.set_vel)

    def followLeftWall(self):
        if regions['front'] < self.distanceO:
        # rotate to the right
            self.turnRight()
        # if the obstacle is on the left, follow the wall
        elif regions['front'] > self.distanceO and regions['left'] < self.distanceO:
            self.followTheWall()

        # if there is no obstacles on front or right, end while loop
        if regions['front'] > self.distanceO and regions['left'] > self.distanceO:
            self.obstacleInFront = False

        self.pub.publish(self.set_vel)

    def callback_classification_result(self, msg):
        return "direction"

    def getDirection(self):
        return "direction"

    # Euclidean distance.
    def get_euclidean_distance(self, x, y):
        return sqrt(pow((self.x - x), 2) + pow((self.y - y), 2))


    # callback function for Odom Subscriber.
    def callback_odometry_msg(self, data):
        qtrn = data.pose.pose.orientation
        [roll, pitch, self.theta] = euler_from_quaternion(
            [qtrn.x, qtrn.y, qtrn.z, qtrn.w])

        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y


    # callback function for Laser Subscriber.
    def callback_laser(self, msg):
        global regions
        regions  = {
            'front': min(min(msg.ranges[0:30]), min(msg.ranges[330:360])),
            'right': min(msg.ranges[255:285]),
            # 'left': min(msg.ranges[75:105]),
            # front angle: 0, (-30 -> 30 = 60)
            # right angle: 270, (30 -> 255 -> 285 = 30)
            # left angle: 90 , (75 -> 105 = 30)
        }
    
    # turning right function
    def turnRight(self):
        self.set_vel.linear.x = 0
        self.set_vel.angular.z = -0.2
        return

    # turning left function
    def turnLeft(self):
        self.set_vel.linear.x = 0
        self.set_vel.angular.z = 0.2

    # moving forward function
    def followTheWall(self):
        self.set_vel.linear.x = 0.2
        self.set_vel.angular.z = 0
        print('wall following')


# ******************************************************************************


if __name__ == '__main__':
    rospy.init_node("robot_control", anonymous=True)

    x = RobotControl()
    x.hedefeGit(0,9)

    rospy.spin()
