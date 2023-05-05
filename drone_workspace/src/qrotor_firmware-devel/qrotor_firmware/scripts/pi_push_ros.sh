#!/bin/bash
rsync -avzh --exclude 'build' --exclude '.git' --exclude '.vscode' --exclude 'docs' --exclude 'qrotor_gazebo'  --exclude 'qrotor_description' /home/kotaru/Workspace/catkin_ws/qrotor_ws/src/qrotor_firmware pi@$1.local:/home/pi/firmware_ws/src/
