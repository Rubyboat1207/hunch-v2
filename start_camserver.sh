#!/bin/bash
gst-launch-1.0 libcamerasrc ! video/x-raw,width=640,height=480,framerate=30/1 \
    ! videoconvert ! v4l2sink device=/dev/video1

