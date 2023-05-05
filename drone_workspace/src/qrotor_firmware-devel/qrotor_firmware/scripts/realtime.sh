#!/bin/bash
echo sudo chrt -f -p 99 \$\(pidof qrotor_NAVIO\)
sudo chrt -f -p 99 $(pidof qrotor_NAVIO)
