#!/usr/bin/env python

# Subsample the images in rosbag by a factor

import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    """Subsample the images in rosbag by a factor    
    """
    parser = argparse.ArgumentParser(description="Subsample the images by a factor")
    parser.add_argument("input_bag_path", help="Input ROS bag.")
    parser.add_argument("output_bag_path", help="Output ROS bag.")
    parser.add_argument("image_topic", help="Image topic.")
    parser.add_argument("factor", type=float, help="Subsample factor")

    args = parser.parse_args()

    print ("Subsample images from %s on topic %s by %s" % (args.input_bag_path,
                                                          args.image_topic, args.factor))

    inbag = rosbag.Bag(args.input_bag_path, "r")
    outbag = rosbag.Bag(args.output_bag_path, "w")
    bridge = CvBridge()
    for topic, msg, t in inbag.read_messages():
        if (topic == args.image_topic):
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            width = int(cv_img.shape[1] * args.factor)
            height = int(cv_img.shape[0] * args.factor)
            cv_img_resized = cv2.resize(cv_img, (width, height))
            
            new_msg = bridge.cv2_to_imgmsg(cv_img_resized, encoding="bgr8")
            new_msg.header = msg.header
            outbag.write(topic, new_msg, msg.header.stamp)
        else:
            outbag.write(topic, msg, msg.header.stamp)
        

    inbag.close()
    outbag.close()

    return

if __name__ == '__main__':
    main()
