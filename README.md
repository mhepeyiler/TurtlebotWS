# TurtlebotWS


All codes were compiled in Ubuntu 20.04 and ROS noetic. To test it, turtlebot3 simulation environment needs to installed in your system. Check the below link. 
The purpose of the repo is to keep improvement process.
In order for any contact please feel to write to me to `murathepeyiler@gmail.com`.

https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/


The project ideas taken from [Programming Robots with ROS: A Practical Introduction to the Robot Operating System. Quigley M, Gerkey B, Smart W. (2015)](https://www.amazon.com/Programming-Robots-ROS-Practical-Introduction/dp/1449323898). The projects in book have been written in Python and earlier version of ROS. Also, it uses turtlebot2. Thus, I have some changes over it. I am doing the project using C++ because I want to improve it and using turtlebot3. 

To build the projects,

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

For all below programs need a turtlebot. So then, please run it before the testing the codes.

## Avoid Obstacle

That is so primative implementation. It is try to avoid object at least try :). To run project,

`rosrun turtlebot_ws turtlebot_ws_avoid_obstacle`

## Tele-op Move

The purpose of the program is to control robot using keyboard. Keyboard layout in below,

```
w - forward
a - turn left
s - stop
d - turtn right
x - backward
i - quit program
```

There are two way to run. First via `rosrun`;

```
rosrun turtlebot_ws turtlebot_ws_keystroke
rosrun turtlebot_ws turtlebot_ws_move_robot
```

or via `roslaunch`

`roslaunch turtlebot_ws keystroke.launch`