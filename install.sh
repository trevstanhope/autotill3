#!/bin/sh

# Aptitude
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install build-essential -y
sudo apt-get install python-serial python-opencv python-pip python-gps -y # python dependencies
sudo apt-get install mongodb -y # MongoDB
sudo apt-get install gpsd gpsd-clients python-gps -y # GPS
sudo apt-get install python-matplotlib -y

# Pip
sudo pip install pymongo
