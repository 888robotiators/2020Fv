#!/bin/bash
cd /home/nvidia/vision2020

echo "starting the posin dude"
while : 
do
    echo "Posing starting up"
    ./PoseEstimation
    #python PoseEstimation.py
    sleep 5
    echo "posing stopped bruh, starting again"
done
