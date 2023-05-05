#!/bin/bash
rsync -avzh --exclude 'build' --exclude '.git' pi@192.168.0.19:/home/pi/qrotor_firmware /home/kotaru/catkin_ws/
