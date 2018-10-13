#!/bin/bash

killall -u root -- python
killall -u root -- mjpg_streamer
sleep 2
rm /root/programmes_python/jpg/*.jpg

/usr/local/bin/mjpg_streamer -i "/usr/local/lib/input_uvc.so -y -f 25" -o "/usr/local/lib/output_http.so -w /usr/local/www" &

python /root/programmes_python/AdressesOLED.py
