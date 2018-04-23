#!/usr/bin/env bash

supervisorctl stop all
supervisorctl shutdown
rosnode kill -a
killall -9 roscore
killall -9 rosmaster
