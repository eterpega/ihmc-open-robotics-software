#!/bin/bash
# Script to extract images from a video frame by frame
#
# To run script: bash extractImagesFromVideo.sh my_video_file_1.extension my_video_file_2.extension my_video_file_3.extension ...

videos="$@"

for i in $videos
do
    echo "Extracting image from: " + ${i}
    mkdir extractedImages-${i}
    ffmpeg -i ${i} extractedImages-${i}/${i}-image%04d.jpg -hide_banner
done

# Delete all files with size zero
# find ~/Akshat/DetectAtlas/Dataset -size  0 -print0 | xargs -0 rm
