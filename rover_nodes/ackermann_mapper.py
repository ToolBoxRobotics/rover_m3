#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from ackermann_msgs.msg import AckermannDrive

WHEEL_RADIUS = 0.062  # meters (example)
WHEEL_BASE = 0.7      # distance between front and rear axles
TRACK_WIDTH = 0.6     # left-right distance

pub = None

def cmdvel_cb(msg: Twist):
    # Convert linear.x and angular.z into wheel speeds and steering angles
    v = msg.linear.x
    omega = msg.angular.z
    # For an ackermann small-angle approximate:
    if abs(omega) < 1e-6:
        steer_angle = 0.0
    else:
        R = v / omega
        steer_angle = math.atan(WHEEL_BASE / R)
    # For 6 wheel rover we set wheel speeds proportional to v
    wheel_speed = v / WHEEL_RADIUS
    data = Float32MultiArray()
    # layout: motor0..motor5 speeds, steerFL, steerFR, steerRL, steerRR
    data.data = [wheel_speed]*6 + [math.degrees(steer_angle)]*4
    pub.publish(data)

def main():
    rospy.init_node('ackermann_mapper')
    rospy.Subscriber('cmd_vel', Twist, cmdvel_cb)
    global pub
    pub = rospy.Publisher('chassis/cmd', Float32MultiArray, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()
