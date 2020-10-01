#!/usr/bin/env python

# Script to convert a 16-bit image to 8-bit
# Used in thermal odometry
# Hengrui (Henry) Zhang

import sys
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def main():
    parser = argparse.ArgumentParser(description='Convert 16-bit raw thermal image to 8-bit image')
    parser.add_argument('input_bag',
                        help='input bag for raw 16-bit data')
    parser.add_argument('output_bag',
                        help='output bag file')
    parser.add_argument('-t', '--topics', default="*",
                        help='string interpreted as a list of topics')

    args = parser.parse_args()

    # setup stuff
    topics = args.topics.split(' ')
    input_bag = rosbag.Bag(args.input_bag, 'r')
    output_bag = rosbag.Bag(args.output_bag, 'w')
    bridge = CvBridge()

    # start to go through the bag
    for topic, msg, t in input_bag.read_messages(topics=topics):
        # read raw image
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        a_max = cv_img.max()
        a_min = cv_img.min()
        cv_img = (cv_img.astype('float32') - a_min) / 2000 * 255;
        cv_img = cv_img.astype('uint8')

        # write 8-bit image msg
        image8_msg = bridge.cv2_to_imgmsg(cv_img, "mono8")
        image8_topic = topic + "_eightbit"
        # fill in headers
        image8_msg.header = msg.header
        output_bag.write(image8_topic, image8_msg, t=t)

    # close bags
    input_bag.close()
    output_bag.close()
    

if __name__ == "__main__":
    main()
