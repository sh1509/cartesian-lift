# cartesian-lift

This project involves simulating and controlling Cartesian motions of a 6-DOF robotic arm mounted on a vertical prismatic joint (lift) using ROS 2 and Ignition Gazebo. It includes velocity-controlled motion, trajectory generation, and optional compliance control.

---

## How to Build and Run the Docker Container

### 1. Build the Docker image

```bash
docker build -t ros2-ignition:humble-citadel .
```

### 2. Run the Docker container with volume mounting

```bash
docker run -it --rm \
  --net=host \
  -v ~/ros2_ws:/workspace \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros2-ignition:humble-citadel
```

---

### Notes

* Before running the container, run `xhost +local:root` on your host to allow GUI forwarding.
* Adjust the volume path `~/ros2_ws` to point to your actual ROS 2 workspace directory.
* This setup assumes you are running on a Linux host with X11. For Windows or Mac, additional configuration may be required.

---
