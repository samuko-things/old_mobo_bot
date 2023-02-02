# mobo_bot

NOTE: development is done in ros2-humble.
clone (git clone git@github.com:samuko-things/mobo_bot.git) or Download
the repo or in your ROS2 workspace, build all packages, and source it.
```
```shell
colcon build --packages-select mobo_bot_description mobo_bot_line_following mobo_bot_nav2d --symlink-install 
```

## Basic Launch
---

![obot](https://github.com/samuko-things/mobo_bot/blob/main/mobo_bot_pic1.png)

```
to view robot in RVIZ:
```shell
$ ros2 launch mobo_bot_description rviz.launch.py
```

to spawn robot in gazebo:
```shell
$ ros2 launch mobo_bot_description sim.launch.py
```

to spawn robot in gazebo and view in RVIZ simultaneously:
```shell
$ ros2 launch mobo_bot_description sim_and_rviz.launch.py
```

you can drive it around using the teleop_twist_keyboard package or you can 
also use the [pynput_teleop_twist_keyboard](https://github.com/samuko-things/pynput_teleop_twist_keyboard/tree/humble_dev) package I wrote.


## Line Follower Mode
---

![obot](https://github.com/samuko-things/mobo_bot/blob/main/mobo_bot_pic2.png)

```
to run line following simulation:
```shell
$ ros2 launch mobo_bot_line_following sim.launch.py
```

then start the follow_line algorithm node:
```shell
$ ros2 run mobo_bot_line_following follow_line.py
```

then send command to the robot: 
first run
```shell
$ ros2 run mobo_bot_line_following command_robot.py
```
then enter 'ff' to followline, 'r'or'l' to turn and track line when it gets to the stop node.