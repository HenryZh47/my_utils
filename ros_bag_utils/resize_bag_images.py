#!/usr/bin/env python

# Script to reduce image size to half 
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
                        help='input bag for original data')
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
    for topic, msg, t in input_bag.read_messages():
        if (topic in topics):
            # read raw image
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # resize
            width = int(cv_img.shape[1] / 2)
            height = int(cv_img.shape[0] / 2)
            dim = (width, height)
            cv_img = cv2.resize(cv_img, dim)

            # write resized image msg
            image_msg = bridge.cv2_to_imgmsg(cv_img, "passthrough")
            image_topic = topic + "_resized"
            # fill in headers
            image_msg.header = msg.header
            output_bag.write(image_topic, image_msg, t=t)
        else:
            output_bag.write(topic, msg, t=t)

    # close bags
    input_bag.close()
    output_bag.close()
    

if __name__ == "__main__":
    main()
