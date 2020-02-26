#!/bin/sh

# Build Docker Image (adjust Dockerfile for your version of Cuda!).
docker build -t autoware-kinetic -f docker/generic/Dockerfile.kinetic .

