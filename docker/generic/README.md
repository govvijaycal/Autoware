# Autoware Docker
To use the Autoware Docker, first make sure the NVIDIA drivers, Docker and nvidia-docker are properly installed.

[Docker installation](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/)


[nvidia-docker installation](https://github.com/NVIDIA/nvidia-docker)

## How to Build
```
$ cd <path_to_Autoware_dir>

# Ubuntu 16.04 (Kinetic)
# Make sure to edit the Dockerfile.kinetic base image for your CUDA version!
$ bash docker/generic/build.sh
```

## How to Run
```

# Ubuntu 16.04 (Kinetic)
$ bash docker/generic/run.sh <-s optional_shared_directory_path>
```

|Option|Default|Description|
|---|---|---|
|-h||Show `Usage: $0 [-s <Shared directory>]`|
|-s|/home/$USER/shared_dir|Specify shared dir|
