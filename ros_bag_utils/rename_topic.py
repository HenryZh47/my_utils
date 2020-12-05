#!/usr/bin/env python

from rosbag import Bag
import argparse


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="rename a topic in rosbag")
    parser.add_argument("bag_path", type=str, default=10, help="rosbag path")
    parser.add_argument("topic_name", type=str, default=10, help="original topic name")
    parser.add_argument("new_topic_name", type=str, default=10, help="new topic name")
    args = parser.parse_args()


    new_bag_path = args.bag_path.split('.')[0] + '_renamed.bag'
    with Bag(new_bag_path, 'w') as Y:
        for topic, msg, t in Bag(args.bag_path):
            Y.write(args.new_topic_name if topic == args.topic_name else topic, msg, t)
