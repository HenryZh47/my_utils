#!/usr/bin/env python3

# Extracts frames from calibration video
# Manually select which frame to save
# Henry Zhang <hzhang0407@gmail.com>

import os
import cv2
import numpy as np
import argparse

def main():
    # argument parsing
    parser = argparse.ArgumentParser(description="extract video frames to a folder")
    parser.add_argument('video_path', type=str, help="video file path")
    parser.add_argument('downsample_rate', type=int, help="downsample rate")
    parser.add_argument('--output_dir', type=str, default="", help="output directory path")
    args = parser.parse_args()

    video_path = args.video_path
    output_dir = args.output_dir
    if (output_dir == ""):
        output_dir = video_path.split('.')[0] + "_frames"
    
    print("extracting frames from video: " + video_path)
    print("extracting images to directory: " + output_dir)

    # print a user instruction
    print("Here are the keys to control the video stream:")
    print("    q: exit the script")
    print("    space: pause video, press any key to continue")
    print("    s: save the frame after the video is paused")
    print("    j: slow down the video by 2x")
    print("    k: speed up the video by 2x")

    # create folder or clear it
    if (os.path.isdir(output_dir)):
        for filename in os.listdir(output_dir):
            file_path = os.path.join(output_dir, filename)
            os.remove(file_path)
    else:
        os.mkdir(output_dir)

    # open video buffer
    cap = cv2.VideoCapture(video_path)
    save_frame_num = 0
    wait_time = 16
    while (cap.isOpened):
        ret, frame = cap.read()
        if not ret:
            break

        cv2.imshow('frame', frame)
        key = cv2.waitKey(wait_time)
        if key == ord('q'):
            # exit
            break
        if key == ord(' '):
            # pause video
            save_key = cv2.waitKey(-1)
            if save_key == ord('s'):
                # save the current frame
                frame_name = output_dir + ("/frame_%d" %(save_frame_num)) + ".jpg"
                save_frame_num += 1

                height, width = frame.shape[:2]
                new_height = int(height / args.downsample_rate)
                new_width = int(width / args.downsample_rate)
                frame = cv2.resize(frame, (new_width, new_height))
                cv2.imwrite(frame_name, frame)
                print("saved frame: %s" %(frame_name))
            else:
                continue
        if key == ord('j'):
            # slow down
            wait_time = wait_time * 2
            print("slow down video with wait time: %d" %(wait_time))
        if key == ord('k'):
            # speed up
            wait_time = int(max(wait_time / 2, 1))
            print("speed up video with wait time: %d" %(wait_time))


    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
