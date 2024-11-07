#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(scan):
    num_readings = len(scan.ranges)
    rospy.loginfo("Received %d laser scan readings", num_readings)
    # 这里可以添加更多处理激光雷达数据的代码

def listener():
    rospy.init_node('lidar_listener', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

