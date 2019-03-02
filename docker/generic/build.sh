#!/bin/sh

# Build Docker Image
if [ "$1" = "9" ]
then
    echo "Use Cuda 9.0"
    docker build -t autoware-kinetic9 -f Dockerfile.cuda9 ./../.. --no-cache
else
    echo "Use Cuda 10.0"
    docker build -t autoware-kinetic10 -f Dockerfile.cuda10 ./../.. --no-cache
fi
