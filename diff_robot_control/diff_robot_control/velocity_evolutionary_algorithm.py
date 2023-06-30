from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray


import rclpy
from rclpy.node import Node

import math

import numpy as np

import sys
sys.path.append("/home/shyngys/simulation_ws/install/diff_robot_control/lib/python3.8/site-packages/diff_robot_control")

from NeuralNet import Net
from GeneticAlgorithm import GeneticAlgorithm


def preprocess(sensors, alpha, time):
    base = 10
    return [base+(base*alpha-base)*(1-math.exp(-i*time)) for i in sensors]

class RobotEAController(Node):

    def __init__(self):
        super().__init__('robot_EA_controller')
        self.last_laser_data = [10]*10
        self.last_dust_count = 0
        self.callback_caller = 0
        self.collision_counter = 0
        self.last_is_collision = False
        god = [  4. ,   4. ,   5. ,   5. ,  -9. ,   5. ,  -3. ,  -4. ,   2. ,
            4. ,  -5. ,   6. ,   3. ,   0. ,   8. ,  10. ,   2. ,  10. ,
            -8. ,   5. ,   8. ,  -2. ,   4. ,   5. ,   6. ,   9. ,   0. ,
            -5. ,   8. ,   0. ,   9. ,   2. ,   0. , -10. ,   8. ,  -8. ,
            8. ,  -2. ,  -2. ,   4. ,   8. ,  -3. ,  -4. ,  -2. ,   2. ,
            0. ,  -1. ,  10. ,  -2. ,   1. ,  -8. ,   2. ,   8. ,   7. ,
            2. ,   3. ,  -4. ,  -6. ,  10. ,  -3. ,   6. ,   8. ,  11. ,
            9. ,   0.2,   0.3]        
        print(god)
        self.cleaningPerformance(god)
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.handleLaser, qos_profile=qos_profile_sensor_data)
        self.dust_pub = self.create_subscription(MarkerArray, 'dust_particles', self.handleDust, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def handleDust(self, msg):
        self.last_dust_count = len(msg.markers)

    def cleaningPerformance(self, genome):
        weights = [np.reshape(genome[:56] ,(4,14))]# first 56
        weights.append(np.reshape(genome[-8:] ,(2, 4))) # last 8
        nn = Net(weights)

        self.nn = nn
        self.alpha = genome[-2]
        self.time = genome[-1]

    def run_simulation(self):
        fitness = 0
        rotated_scan = [self.last_laser_data[9]] + self.last_laser_data[:9]
        processed_scan = preprocess(rotated_scan, self.alpha, self.time)
        # print("processed_scan: ", processed_scan)
        output = self.nn.feedForward(processed_scan)
        # post process 
        v_l = output[0]
        v_r = output[1]
        
        print(v_l, v_r)
        self.sendVelocities(v_l, v_r)

    def handleLaser(self, msg):
        # print(self.callback_caller)
        if self.callback_caller % 5 == 0:
            self.last_laser_data = [100 if x == np.inf else x for x in msg.ranges]
            self.run_simulation()
        self.callback_caller += 1

    def stopRobot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.vel_pub.publish(twist)

    def sendVelocities(self, v_l, v_r):
        speed = 0.03
        turn = 0.3
        wheelSeparation = 0.4
        wheelDiameter = 0.2

        if v_r == v_l:
            twist = Twist()
            twist.linear.x = v_r * speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.vel_pub.publish(twist)
        else:
            sr = v_r*wheelDiameter/2
            sl = v_l*wheelDiameter/2
            dtheta = (sr - sl)/wheelSeparation
            twist = Twist()
            twist.linear.x = (v_r+v_l)/2 * speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = dtheta * turn
            self.vel_pub.publish(twist)

def main():
    rclpy.init()
    node = RobotEAController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stopRobot()
        pass

    rclpy.shutdown()