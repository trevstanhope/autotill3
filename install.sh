#!/bin/sh

# Aptitude
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install build-essential cmake -y
sudo apt-get install python-serial python-pip python-gps -y # python dependencies
sudo apt-get install mongodb -y # MongoDB
sudo apt-get install gpsd gpsd-clients python-gps -y # GPS
sudo apt-get install python-matplotlib -y

## OpenCV
wget http://downloads.sourceforge.net/project/opencvlibrary/opencv-unix/2.4.9/opencv-2.4.9.zip
unzip opencv-2.4.9.zip
cd opencv-2.4.9
mkdir release
cd release
cmake -D CV_BUILD_TYPE=RELEASE ..
make -j4
sudo make install

## Pip
sudo pip install pymongo
