#!/bin/sh

# Build Docker Image
if [ "$1" = "kinetic" ] || [ "$1" = "indigo" ]
then
    echo "Use $1"
    cd ../../ && nvidia-docker build -t autoware-$1 -f docker/generic/Dockerfile.$1 .
else
    echo "Select distribution, kinetic|indigo"
fi
