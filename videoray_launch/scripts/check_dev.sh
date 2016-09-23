#!/bin/bash

v4l2-ctl -s ntsc --device=1

if [ -e /dev/sonar ]; then
	echo "Device sonar found"
	sudo chmod 777 /dev/sonar
else
	echo "Device sonar not found"
fi

if [ -e /dev/videoray ]; then
	echo "Device VideoRay found"
	sudo chmod 777 /dev/videoray
else
	echo "Device VideoRay not found"
fi

if [ -e /dev/usbl ]; then
	echo "Device USBL found"
	sudo chmod 777 /dev/usbl
else
	echo "Device USBL not found"
fi

if [ -e /dev/input/videorayjs ]; then
	echo "Device joystick found"
	sudo chmod 777 /dev/input/videorayjs
else
	echo "Device joystick not found"
fi
