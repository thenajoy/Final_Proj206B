#!/bin/bash
#rsync -avzh --exclude 'build' --exclude '.git' --exclude '.vscode' /home/kotaru/workspace/catkin_ws/qrotor_firmware pi@navio.local:/home/pi
rsync -avzh --exclude 'build' --exclude '.git'  --exclude '.vscode' /home/kotaru/workspace/git/vkotaru/qrotor_firmware pi@navio.local:/home/pi

