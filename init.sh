#!/bin/sh
sudo gpsd /dev/ttyS0
cd /root/autotill3
python autotill3.py
