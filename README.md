# ReadMe
## Note:
1. Install `Gazebo Harmonic` and `ROS Jazzy`
2. Install `PX4-Autopilot` version `v1.15.0`

```
mkdir -p vtca/px4_swarm_ws/src
cd ~/vtca/px4_swarm_ws/src 
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout tags/v1.15.0 -f

# syncs all submodules to the same version as required by v1.15.0 
git submodule sync --recursive
git submodule update --init --recursive

# Install PX4-Autopilot
~/vtca/px4_swarm_ws/src/PX4-Autopilot/Tools/setup/ubuntu.sh –no-sim-tools

# Try installation, this should spin up a Gazebo Harmonic window with an x500 drone.
cd ~/vtca/px4_swarm_ws/src/PX4-Autopilot
make px4_sitl gz_x500
```

3. Install  `px4-ros-com` and `px4-msgs repo` and checkout `release/1.15`

```
cd ~/vtca/px4_swarm_ws/src
git clone https://github.com/PX4/px4_msgs.git 
git checkout release/1.15 # switch to releases/1.15 branch

cd ~/vtca/px4_swarm_ws/src
git clone https://github.com/PX4/px4_ros_com.git

sudo rosdep init
rosdep update

# From the src directory. Only install px4_msgs & px4_ros_com.
colcon build --executor sequential --symlink-install --packages-up-to px4_msgs px4_ros_com

```
4. Clone and build Micro-XRCE-DDS-Agent and launch it with the following command: `MicroXRCEAgent udp4 -p 8888`

```
cd ~/vtca/px4_swarm_ws/src 
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake
make
sudo make install 

# To run it:
cd ~/vtca/px4_swarm_ws/src/Micro-XRCE-DDS-Agent
export LD_LIBRARY_PATH=/home/mkk87/vtca/px4_swarm_ws/src/Micro-XRCE-DDS-Agent/build:$LD_LIBRARY_PATH

# Run it on a screen, so it can run in the background
/usr/local/bin/MicroXRCEAgent udp4 -p 8888 

```

5. Install Drone Patrol
```
cd ~/vtca/px4_swarm_ws/src/
git clone -b implement-multidrone-patrolling https://github.com/shravankumargulvadi/drone-patrolling.git 

# Install
source ~/vtca/px4_swarm_ws/install/setup.bash
cd ~/vtca/px4_swarm_ws/src/drone-patrolling
colcon build
```

6. Multi-Drone Simulation

```
# Build PX4 SITL
cd ~/vtca/px4_swarm_ws/src/PX4-Autopilot
make px4_sitl 

# Kill existing instances if any 
pkill -9 px4
pkill -9 gzclient
pkill -9 gzserver 

# Start multiple drone simulation (each in new terminal)

PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1

PX4_SYS_AUTOSTART=4001 PX4_GZ_STANDALONE=1 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 2

# Assume Micro-XRCE-DDS is runnning on 8888

# Start drone manager - 1 (New terminal)

cd ~/vtca/px4_swarm_ws/src/drone-patrolling
source install/setup.bash
PX4_UXRCE_DDS_PORT=8888 ros2 run drone_control drone_manager --ros-args -p drone_id:=1

# Start drone manager - 2 (New terminal)

cd ~/vtca/px4_swarm_ws/src/drone-patrolling
source install/setup.bash
PX4_UXRCE_DDS_PORT=8888 ros2 run drone_control drone_manager --ros-args -p drone_id:=2

# Start Ground controller  (New terminal)

cd ~/vtca/px4_swarm_ws/src/drone-patrolling
source install/setup.bash
ros2 run drone_control ground_control_node --ros-args -p drone_ids:='["px4_1", "px4_2"]'
```

