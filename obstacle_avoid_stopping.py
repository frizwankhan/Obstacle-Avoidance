#!/usr/bin/env python

import rospy
import argparse

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

value = False
lidar = LaserScan()
pose = PoseStamped()
vel_0 = Twist()
vel_0.linear.x = 0
vel_0.linear.y = 0
vel_0.linear.z = 0

parser = argparse.ArgumentParser(description="setpoint/postition")
parser.add_argument('-x', '--x', type=int, help="x component of point")
parser.add_argument('-y', '--y', type=int, help="y component of point")
parser.add_argument('-z', '--z', type=int, help="z component of point")
args = parser.parse_args()

point = PoseStamped()
point.pose.position.x = args.x
point.pose.position.y = args.y
point.pose.position.z = args.z

value = False


def function(msg):
    global lidar
    lidar = msg
    global value
    for i in range(470,530):
        angle = (180*i)/512
        print(f"angle = {angle} i = {i} range = {lidar.ranges[i]}")
       
        if lidar.ranges[i]<7 and lidar.ranges[i]>1:
            print(f'range = {lidar.ranges[i]}')
            for i in range(0,1000):
                pub.publish(vel_0)
                
                print('publishing')

            value = True
            break
    


def function1(msg):
    global pose
    pose = msg





    
    
    
        




rospy.init_node('node')
pub1 = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
for i in range(0,10000000):
    continue
pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped',Twist, queue_size=10)
print('done1')
sub = rospy.Subscriber('/spur/laser/scan', LaserScan, function)

for i in range(0,10000000):
    continue

sub1 = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, function1)

print('done2')


rate = rospy.Rate(5)

while not rospy.is_shutdown():
    rate.sleep()
    pub1.publish(point)
    if value:
        for i in range(0,100):
            pub.publish(vel_0)
        for i in range(0,10):
            pub1.publish(pose)
        break



print('mission completed')

