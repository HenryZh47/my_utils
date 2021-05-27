#!/bin/bash
# bash data_transform.sh data_folder video_name
mkdir /data/$1

mv /data/$2_sink.mkv /data/$1
mv /data/$2_subtitle.srt /data/$1
cp /data/*.bag /data/$1/imu.bag 
rm -rf /data/*.bag 

python vid_srt_to_bag.py -m /data/$1 -c /data/$1 -o /data/$1 -ds 3

python mergebag.py -o /data/$1/test.bag -i /data/$1/$2_sink.bag /data/$1/imu.bag
