#! /usr/bin/python

import rospy
import time

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from math import pow, atan2, sqrt, pi

turtles = list()

def kill_handler():
    rospy.wait_for_service('/kill') 
    kill_func = rospy.ServiceProxy('/kill', Kill)
    for turtle in turtles:
        kill_func(turtle.name)
        print "\nKill " + turtle.name 


class Turtle_chaser:
    def __init__(self, name, coord, speed):
        self.name = name
        self.speed = speed
        rospy.wait_for_service('/spawn')
        spawn_func = rospy.ServiceProxy('/spawn', Spawn)
        spawn_func(coord[0], coord[1], coord[2], name)
        self.pub = rospy.Publisher(name + '/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('turtle1/cmd_vel', Twist, self.update_direction)
        self.sub2 = rospy.Subscriber(name + '/pose', Pose, self.update_position)
        self.sub3 = rospy.Subscriber('turtle1/pose', Pose, self.update_goal_position)
    
    def update_goal_position(self, data):
        self.goal_pose = data

    def update_position(self, data):
        self.pose = data

    def update_direction(self, msg):
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = self.angular_vel(self.goal_pose)
        msg.linear.x = self.linear_vel(self.goal_pose)
        msg.linear.y = 0
        msg.linear.z = 0
        self.pub.publish(msg)
    
    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                        pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, private_space=1): 
        return self.speed if (self.euclidean_distance(goal_pose) > private_space) else 0

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=2,private_space=1):
        if (self.euclidean_distance(goal_pose) < 1):
            return 0
        else:
            return constant * (self.steering_angle(goal_pose) - self.pose.theta)


rospy.init_node('more_turtles')

turtles.append(Turtle_chaser(name='Bunny', coord=(1.0, 1.0, 0.0), speed=1.0))
turtles.append(Turtle_chaser(name='Tanny', coord=(1.0, 10.0, 0.0), speed=2.5))

rospy.spin()
