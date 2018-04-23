#!/usr/bin/env bash
echo "Found container ID: $(docker ps -lq)"
docker commit $(docker ps -lq) twizy/apollo
echo "Saved!"
