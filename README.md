# mobo_bot

---

![obot](https://github.com/samuko-things/mobo_bot/blob/main/mobo_bot_pic1.png)

```
NOTE: devvelopment is done in ros2-humble.
clone (git clone git@github.com:samuko-things/mobo_bot.git) or Download
the repo or in your ROS2 workspace, build, and source it.
```
## Basic Launch
to view robot in RVIZ:
```shell
$ ros2 launch mobo_bot_description view.launch.py
```

to spawn robot in gazebo:
```shell
$ ros2 launch mobo_bot_description spawn.launch.py
```

to spawn robot in gazebo and view in RVIZ simultaneously:
```shell
$ ros2 launch mobo_bot_description spawn_and_view.launch.py
```

you can drive it around using the teleop_twist_keyboard package or you can 
also use the [pynput_teleop_twist_keyboard](https://github.com/samuko-things/pynput_teleop_twist_keyboard/tree/humble_dev) package I wrote.
