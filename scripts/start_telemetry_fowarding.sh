#!/bin/bash
echo "Started mavlink..."
echo "skyhigh" | sudo -S chmod 666 /dev/ttyTHS1
mavproxy.py --master /dev/ttyTHS1 --baudrate 921600 --out 127.0.0.1:14540 --out 127.0.0.1:14541 --out 172.25.213.195:14550 --out 172.25.151.221:14551