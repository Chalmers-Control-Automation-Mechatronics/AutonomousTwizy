#!/usr/bin/env bash

bash /apollo/scripts/bootstrap.sh
supervisorctl start canbus control gps localization perception planning prediction routing usb_camera velodyne
