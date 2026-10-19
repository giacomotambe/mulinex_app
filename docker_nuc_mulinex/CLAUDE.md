# CLAUDE.md

## Project Overview
Docker development environment for the **Mulinex** omnidirectional robot, targeting an Intel NUC running ROS 2 Humble. The container is based on `osrf/ros:humble-desktop-full` and provides a full ROS 2 workspace with GUI support (X11 forwarding), NVIDIA GPU passthrough, and tools like PlotJuggler, Gazebo (Ignition Fortress), and rviz2.

## Docker Usage
Build, start, attach, and stop the container with: `make build`, `make start`, `make attach`, and `make stop` respectively. The container is named `mulinex` and the image is `mulinex:humble`.

## Architecture

### ROS 2 Packages Description
For an overview of the packages:
```shell
for pkgxml in src/*/package.xml; do \
  name=$(grep -oP '(?<=<name>).*?(?=</name>)' "$pkgxml"); \
  desc=$(grep -oP '(?<=<description>).*?(?=</description>)' "$pkgxml"); \
  echo "$name: $desc"; \
done
```

## Key Configuration Details
- **ROS_DOMAIN_ID**: 10
- **RMW_IMPLEMENTATION**: `rmw_cyclonedds_cpp`
