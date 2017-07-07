#!/bin/bash
# Script to extract images from a video frame by frame
#
# To run script: bash extractImagesFromVideo.sh my_video_file_1.extension my_video_file_2.extension my_video_file_3.extension ...

videos="$@"

for i in $videos
do
    file=${i%.*}
    file=${file##*/}
    tput setaf 1
    echo "Extracting image from: ${file}"
    tput sgr0
    mkdir -p extractedImages-${file}
    ffmpeg -i ${i} extractedImages-${file}/${file}-image%04d.jpg -hide_banner
done
