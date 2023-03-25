#!/bin/bash
sleep 1
rosrun mor_localization clear_terminal.sh > /dev/null 2>&1 &
clear
echo "############################################# Particle filter in 2D localization $(date -R) ###################"
read
