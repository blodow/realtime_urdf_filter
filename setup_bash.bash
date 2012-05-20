#!/bin/bash

export PKG_CONFIG_PATH=/home/f_asimov/packages/ros/vision_opencv/opencv2/opencv/release/lib/pkgconfig
cmake -DOpenCV_DIR=/home/f_asimov/packages/ros/vision_opencv/opencv2/opencv/release/ .
