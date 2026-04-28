# Note
This version of code is for vertical configuration of the WAM teleop setup. IGNORE the rest of the branches for now.

# Wam Teleop
This package enables teleoperation between the 4DOF leader with the haptic wrist and the 7DOF follower. While this is a ros package, communication between WAMs does not use ros, and instead uses UDP. Ros is only used to publish the state of arm for easier data collection, and is not intended to receive any incoming messages or services to control the arm. 

## Build Instructions

Place this package in `<catkin_ws>/src/`.

By default, this package builds the leader and follower node. If you only wish to build the follower node you can configure this from the command line:
```bash
catkin_make --cmake-args -DBUILD_LEADER=OFF 
```
or set the option to `OFF` in `CMakeLists.txt`:
```bash
option(BUILD_LEADER "Build leader executable" OFF)
```
To build the leader node, the haptic_wrist library is required as a dependency, build and install instructions can be found [here](https://github.com/dmiller12/libhaptic_wrist).

OR, to make your life easy, see [wam-ros-docker](https://github.com/ualberta-robotics/wam-ros-docker) where you only need to run ```source build_all.sh``` or ```source build_ws.sh``` if you already built the gripper and wrist.

## Run Instructions

`config/` contains the Barrett configuration files for the leader and follower. 
You can set the correct config file by using `source scripts/setup_leader.sh` and `source scripts/setup_follower.sh`. These will set the env variable `BARRETT_CONFIG_FILE` to the correct path. 
You may need to modify the bus port in `config/leader.conf` and `config/follower.conf` depending on the can interface. Note that these environment variables only persist for the current terminal session.

to run each node:
```bash
rosrun wam_teleop leader
rosrun wam_teleop follower
```
see teleop_config.yaml to change ports, hosts, gains and more!

In your host computer, run 
```bash
source scripts\can_init_pciefd.sh
```
or,
```bash
source scripts\can_init_usb.sh
```
or, edit to your needs depending on how you communicate with the wam's.

NOTE: you might have to tweak the can# based on the order you plugged stuff into the computer.

In a separate terminal session, start the master node with: `roscore`.

In a separate terminal session, start the leader in `wam_ws\src\wam_teleop`:
```bash
source scripts/setup_leader.sh
rosrun wam_teleop leader
```
In another separate terminal session, start the follower in `wam_ws\src\wam_teleop`:
```bash
source scripts/setup_follower.sh
rosrun wam_teleop follower
```

Once both nodes have started:

1) On the leader use `l` to go to the sync position.
2) On the follower use `l` to go to the sync position. Ensure both arms have reached the sync position before continuing.
3) Press enter to link leader
4) Press enter to link follower

The arm is now ready for user teleoperation.

To turn off, it is recommended to go through the following procedure to ensure proper thread and socket cleanup.
1) Return both wams to home position
2) On the leader, press `x` to exit the loop.
3) Shift idle the leader
4) Repeat for follower. Press `x` to exit the loop
5) Shift idle the follower.
