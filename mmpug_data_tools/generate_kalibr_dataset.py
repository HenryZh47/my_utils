#!/usr/bin/env python3

from shutil import copyfile, move
import os
import sys
import argparse

parser = argparse.ArgumentParser(description="change video frames to file names with timestamp")
parser.add_argument("subtitle_file", help="timestamp subtitle file")
parser.add_argument("frames_folder", help="calibration frames")
parser.add_argument("output_folder", type=str, default="", help="output frames folder with timestamp as file name")

args = parser.parse_args()

SUBTITLE_FILE = args.subtitle_file
FRAMES_FOLDER = args.frames_folder
OUTPUT_FOLDER = args.output_folder
INPUT_FRAME_RATE = 24
OUTPUT_FRAME_RATE = 4
START_FRAME_NUM = 1  # 16+1


with open(SUBTITLE_FILE, 'r') as f:
    lines = f.readlines()
    n = len(lines)
    skip_frames = int(INPUT_FRAME_RATE / OUTPUT_FRAME_RATE)
    last_i = (START_FRAME_NUM-1) * 8
    for i in range((START_FRAME_NUM-1) * 8, n):
        if lines[i] == '\n':
            frame_num = int(lines[last_i].strip('\n'))
            if (frame_num - START_FRAME_NUM) % skip_frames != 0:
                last_i = i + 1
                continue
            print(frame_num)
            if i-last_i != 7:
                print('Using RT for frame num', frame_num)
                parts = lines[last_i+2].split()
            else: 
                parts = lines[last_i+5].split()
            if len(parts[2]) < 9:
                 parts[2] = (9 - len(parts[2])) * '0' + parts[2]
            tc = parts[1] + parts[2]
            assert (len(tc) == 19)
            last_i = i + 1
            src = os.path.join(FRAMES_FOLDER, str(frame_num) + '.png')
            dst = os.path.join(OUTPUT_FOLDER, tc + '.png')
            if os.path.exists(src):
                # os.rename(src, dst)
                # copyfile(src, dst)
                move(src, dst)
