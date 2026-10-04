# Note
This version of code is for the end effector wrist ![EE wrist](https://github.com/ualberta-robotics/wam-ros-docker/blob/main/media/ee_wrist.jpg).


# Description
Useful features for the WAM robots.

- Teleoperation supporting different wrists and grippers.
- Run with policy (separate repos). Interpolate received action chunks to 500hz and execute with variable gain shared control.
- Dynamic compensation
- Logging

Communication is done through udp. See `src/lib/udp/`.


## Run Instructions

`config/` contains the Barrett configuration files for the leader and follower. 
You can set the correct config file by using `setup_leader` and `setup_follower` if using [wam-ros-docker](https://github.com/ualberta-robotics/wam-ros-docker) ;)
These will set the env variable `BARRETT_CONFIG_FILE` to the correct path. You may need to modify the bus port in `config/leader.conf` and `config/follower.conf` depending on the can interface. Note that these environment variables only persist for the current terminal session.

to run each node:
```bash
rosrun wam_teleop leader
rosrun wam_teleop follower
```
to change what is sent between robots, and more see:
    teleop_config.yaml
    policy_config.yaml
    logging_config.yaml
   

In your host computer, run 
```bash
source scripts\can_init_pciefd.sh
```
or,
```bash
source scripts\can_init_usb.sh
```
or, edit to your needs depending on how you communicate with the wam's.

NOTE: you might have to tweak the can# based on the order you plugged the can cables into the computer.

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

## Starting teleoperation

Once both nodes have started:

1) On the leader use `l` to go to the sync position.
2) On the follower use `l` to go to the sync position. Ensure both arms have reached the sync position before continuing.
3) Press enter to link leader
4) Press enter to link follower

Linking is refused if the two arms are not within `link_tolerance` rad of each
other (see `config/teleop_config.yaml`).

## Starting Policy
1) On either leader or follower press `p` and enter.
2) TODO: To load the policy automatically  without cli confirmation
3) TODO: Provide python udp template for easy setup

## Starting Dynamics
1) On either leader or follower press `d` and enter.
2) TODO: To load the dynamics automatically  without cli confirmation

## Starting Logging
1) On either leader or follower press `g` and enter.
2) TODO: To load the logging automatically  without cli confirmation
3) See `config/logging_config` for what gets printed


## Full Command List
* `l` - link/unlink the teleop module
* `p` - toggle the policy module
* `d` - toggle the dynamics module
* `g` - toggle the logging module
* `t` - tune WAM JP control gains
* `x` - exit

Loaded modules that produce torque (policy, dynamics) are summed into the final
control torque. The dynamics control law is shared by both sides and set with
`dynamics.law` in `config/policy_config.yaml`; which fields the logging module
prints is set in `config/logging_config.yaml`.

To turn off, it is recommended to go through the following procedure to ensure proper thread and socket cleanup.
1) Return both wams to home position
2) On the leader, press `x` to exit the loop.
3) Shift idle the leader
4) Repeat for follower. Press `x` to exit the loop
5) Shift idle the follower.
