#!/bin/bash

v4l2-ctl --list-devices
read -p "Enter Camera device number:" camera_device
echo selected camera is: $camera_device
v4l2-ctl --device /dev/video$camera_device --set-ctrl=exposure_auto=1
#v4l2-ctl --device /dev/video$camera_device --set-ctrl=exposure_absolute=400

v4l2-ctl --device /dev/video$camera_device --set-ctrl=exposure_absolute=200

v4l2-ctl --device /dev/video$camera_device --get-ctrl=exposure_absolute
