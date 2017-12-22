#!/bin/bash

make
du heightControl2017.bin -h 
cp heightControl2017.bin /media/jeroen/MBED/heightControl2017.bin -f
sync
read -n 1 -s -p "Press any key to continue"
minicom -D /dev/ttyACM0 -b 9600 #115200
