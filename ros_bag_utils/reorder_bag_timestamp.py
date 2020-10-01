#!/usr/bin/python3
import rosbag
import rospy
import sys

inputbag = sys.argv[1]
outputbag = sys.argv[2]
print("Input:  %s" % inputbag)
print("Output: %s" % outputbag)

with rosbag.Bag(outputbag, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(inputbag).read_messages():
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        if topic == "/tf" and msg.transforms:
            outbag.write(topic, msg, msg.transforms[0].header.stamp)
        else:
            outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
