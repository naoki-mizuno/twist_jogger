# twist_jogger

Control your manipulator in any given frame, using only Twist messages.

## What's This?

`twist_jogger` lets you jog the end-effector of your manipulator with Twist
messages.

## Demo

Clone and install
[`twist_jogger_demo`](https://github.com/naoki-mizuno/twist_jogger_demo) to
your workspace. Requirements for running this demo are:

- [`universal_robot`](https://github.com/ros-industrial/universal_robot)
- [`mofpy`](https://github.com/naoki-mizuno/mofpy)
- [`mofpy_demo`](https://github.com/naoki-mizuno/mofpy_demo)
- a PS4 controller (not necessary, but you'll need to change the
  configurations in `mofpy` if you're using some other joypads)

If you have your own ROS node for publishing `TwistStamped` to jog
manipulators, feel free to use that.

```
$ roslaunch twist_jogger_demo ur5.launch
```

This will bring up Gazebo with the UR5 manipulator loaded. You can control the
end-effector in the world frame as follows (if you're using a PS4 controller
with `mofpy`):

- X: left vertical stick
- Y: left horizontal stick
- Z: L2, R2
- pitch: right vertical stick
- roll: right horizontal stick
- yaw: L1, R1

## Features

Here are the features of `twist_jogger` that you might like.

### Changing the frame of command

The demo let you control the end-effector in the world frame. If you want to
control the end-effector in the end-effector frame (this is useful when you're
doing teleop through a camera mounted on the end-effector), simply change the
`frame_id` of the twist messages that are being sent to `twist_jogger`. This
also implies that you can dynamically change the frame of command without
restarting the node by simply modifying the value in `frame_id`.

### Stable execution on actual robot

`twist_jogger` enables stable execution of the commanded movement by taking
into account the fluctuations in the current joint values sent from the
motors. It does this by keeping the "ideal" state of the joints and commanding
the movements as if the previous commands were perfectly executed.

We use `twist_jogger` on our 6DOF manipulator and have found that it works
stably both on the simulator and on the actual robot.

### Works on any manipulator using MoveIt!

`twist_jogger` uses the `/compute_ik` and `/compute_fk` services provided by
MoveIt! You can use `twist_jogger` on a 6DOF manipulator as well as 7DOF. You
can also test out different inverse kinematics solver, and you can even use
your own solver using IKFast.

## Using it on your robot

In order to use `twist_jogger` on your robot, you'll need to make your robot
compatible with MoveIt! (more specifically, have the `/compute_ik` and
`/compute_fk` services available).

## Similar projects

- [`jog_arm`](http://wiki.ros.org/jog_arm)
- [`jog_control`](https://github.com/tork-a/jog_control)

## License

MIT

## Author

Naoki Mizuno (mizuno.naoki@rm.is.tohoku.ac.jp)
