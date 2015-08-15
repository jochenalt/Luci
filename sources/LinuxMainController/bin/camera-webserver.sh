#!/bin/bash

echo "starting mjpg_streamer..."
PORT=8001
FILEPATH=/home/odroid/camera


echo Webserver with face runs at http://`ifconfig usb0 | awk '/inet addr/ { sub(/addr:/,""); print $2}'`:$PORT/javascript.html displaying $FILEPATH/*.jpg

mjpg_streamer -i "/usr/local/lib/input_file.so -f $FILEPATH -r" -o "/usr/local/lib/output_http.so -w /usr/local/www -p $PORT"  

