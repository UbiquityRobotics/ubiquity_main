
# Magni Robot Application Programming Intefaces

## ROS

The non-embedded parts of the Magni Robot System are implemented on
top of the [Robot Operating System](http://wiki.ros.org/).

A ROS system is implemented as a number of processing _nodes_, which
run as processes on one or more connected computers.  Communication
between the nodes is performed over _topics_, which are named and
typed data channels.

Tools to visualize the connections between nodes are described in the
[topic tutorial](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics).

ROS provides a number of [client libraries](http://wiki.ros.org/Client%20Libraries), the most commonly used of which are C++ and Python.

There are a large number of [tutorials](http://wiki.ros.org/ROS/Tutorials)
available, that cover installing ROS, using it, and programming new
functionality.

## Motor Control Messages

Speed commands can be sent to the motor via the `/cmd_vel` topic which is of
type [`Twist`](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)

A Twist message can be sent from the command line using the `rostopic` utility

```
rostopic pub /cmd_vel geometry_msgs/Twist '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
```

In the example above, the robot is instructed to move forward at 1 meter/second.
The second example shows a command to rotate the robot ant-clockwise at
1 radian/second.

```
rostopic pub /cmd_vel geometry_msgs/Twist '[0.0, 0.0, 0.0]' '[0.0, 0.0, 1.0]'
```
