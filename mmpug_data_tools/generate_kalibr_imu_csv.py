#!/usr/bin/env python

from sensor_msgs.msg import Imu
import rospy
import argparse

parser = argparse.ArgumentParser(description="extract IMU to a csv")
parser.add_argument("imu_topic", help="IMU topic for extraction")
parser.add_argument("output_file", help="output path for the csv file")
args = parser.parse_args()

# IMU_TOPIC = '/gq7/nav/filtered_imu/data'
IMU_TOPIC = args.imu_topic

def callback(imu_data, args):
    f = args[0]
    stamp = imu_data.header.stamp
    wx = imu_data.angular_velocity.x
    wy = imu_data.angular_velocity.y
    wz = imu_data.angular_velocity.z
    ax = imu_data.linear_acceleration.x
    ay = imu_data.linear_acceleration.y
    az = imu_data.linear_acceleration.z
    f.write('{},{},{},{},{},{},{}\n'.format(stamp, wx, wy, wz, ax, ay, az))
    rospy.loginfo('{},{},{},{},{},{},{}\n'.format(stamp, wx, wy, wz, ax, ay, az))


def main():
    f = open(args.output_file, "w")
    rospy.init_node('xavier_data_engine', anonymous=True)
    rospy.Subscriber(IMU_TOPIC, Imu, callback, [f])
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        pass
