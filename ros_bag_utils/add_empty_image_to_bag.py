#!/usr/bin/env python

# add black (empty) image every N frames

import os
import argparse
import cv2
import numpy as np
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    parser = argparse.ArgumentParser(description="Insert black image every N frame")
    parser.add_argument("input_bag", help="Input ROS bag.")
    parser.add_argument("image_topic", help="Image topic.")
    parser.add_argument("--output_bag", type=str, default="", help="Output ROS bag.")
    parser.add_argument("--n_frame", type=int, default=1000, help="empty image interval (frames)")
    parser.add_argument("--duration", type=int, default=10, help="empty image duration (frames)")

    args = parser.parse_args()

    input_bag = rosbag.Bag(args.input_bag, "r")
    if (args.output_bag != ""):
        output_bag_path = args.output_bag
    else:
        name_list = args.input_bag.split(".")
        output_bag_path = "".join(name_list[:-1]) + "_blanked.bag"
    output_bag = rosbag.Bag(output_bag_path, "w")
    print ("Inserting blank images to rosbag: %s, on topic: %s, output to: %s" % (args.input_bag,
                                                          args.image_topic, output_bag_path))

    bridge = CvBridge()

    frame_count = 0
    blanked_frame_count = 0
    do_blank = False
    for topic, msg, t in input_bag.read_messages():
        if (topic == args.image_topic):
            if (do_blank):
                # target image topic
                # create an empty image
                img_height = msg.height
                img_width = msg.width
                blank_img = np.zeros((img_height, img_width, 1), np.uint8)

                # create msg
                new_msg = bridge.cv2_to_imgmsg(blank_img, msg.encoding)
                new_msg.header = msg.header

                # write msg to new bag
                output_bag.write(topic, new_msg, t)

                # update counter
                blanked_frame_count += 1
                if (blanked_frame_count % args.duration == 0):
                    do_blank = False
                    blanked_frame_count = 0
            else:
                # write msg to new bag
                output_bag.write(topic, msg, t)

            frame_count += 1
            if (frame_count % args.n_frame == 0):
                do_blank = True
                frame_count = 0
        else:
            # other topics, just write through
            output_bag.write(topic, msg, t)

if __name__ == "__main__":
    main()
