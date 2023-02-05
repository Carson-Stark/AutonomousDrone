#!/bin/bash

echo "Started mavlink..."
sudo chmod 666 /dev/ttyTHS1
mavproxy.py --master /dev/ttyTHS1 --baudrate 921600 --out 127.0.0.1:14540 --out 172.25.151.221:14550