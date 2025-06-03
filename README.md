# ReadMe
## Note:
1. Clone `PX4-Autopilot` repository and checkout `v1.15.0`
2. Clone `px4-ros-com` and `px4-msgs repo` and checkout `release/1.15`
3. Clone and build Micro-XRCE-DDS-Agent and launch it with the following command: `MicroXRCEAgent udp4 -p 8888`
4. Use the following command the ROS node that controls the drone: `ros2 run drone_control drone_manager`
