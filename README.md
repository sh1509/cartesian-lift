# cartesian-lift

This project involves simulating and controlling Cartesian motions of a 6-DOF robotic arm mounted on a vertical prismatic joint (lift) using ROS 2 and Ignition Gazebo. It includes velocity-controlled motion, trajectory generation, and optional compliance control.

---

## How to Build and Run the Docker Container

### 1. Build the Docker image

```bash
$ cd docker
$ docker compose -f docker-compose.sim.yaml build
```

### 2. Run the Docker container

```bash
cd docker
$ docker compose -f docker-compose.sim.yaml up
```

---

### Notes

* Before running the container, run `xhost +local:root` on your host to allow GUI forwarding.
* This setup assumes you are running on a Linux host with X11. For Windows or Mac, additional configuration may be required.

---

## Dependencies

This project depends on the following:

* **Universal\_Robots\_ROS2\_Description**
  Included as a git submodule under the `external/` directory.
  To initialize and update it:

  ```bash
  git submodule update --init --recursive
  ```

* **ROS 2 Humble** with the following packages:

  * `gazebo_ros_pkgs`
  * `gazebo_ros2_control`
  * `ros_ign`
  * `xacro`
  * `rviz2`
  * `joint_state_publisher_gui`

* **Gazebo Fortress (Ignition Gazebo)**
  Installed via OSRF APT repository:

  ```Dockerfile
  curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list
  apt-get update && apt-get install -y ignition-fortress
  ```

* **Python packages**

  * `matplotlib` (for plotting velocities etc.)

> All dependencies are installed in the provided Dockerfile.

---

