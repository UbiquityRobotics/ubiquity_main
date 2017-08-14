
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

In ROS differential drive robots are controlled by sending speed commands to the motor controller.
Speed commands can be sent to the motor via the `/cmd_vel` topic which is of
type [`Twist`](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html).

In essence a twist command is a request for the motors to go at a certain speed.
This request is not unlimited, ROS assumes that the motor controller has a 
dead-man timer, which will stop the motors if no new `/cmd_vel` message is sent to 
the motor controller in a certain period of time. That way if a process dies or something
goes wrong the robot will come to rest and remain in a safe state. 

In Magni's motor controllers the dead-man timer is, by default, set to 100ms. A Magni motor
controller is position oriented - in that it tries to achieve the position (not the speed) that 
it expects the process wanted. When a cmd_vel message is received by the motor controller, it 
assumes that the intent of the program is to get to a target position by the time the deadman timer 
expires. The target position is the position that would be achieved if the motor-controller went at
exactly the cmd_vel message speed for the time it takes the motor controller went at the cmd_vel 
speed. Ten thousand times per second the motor controller computes the position that the wheels 
would need to be in order to be on a linear path to getting to that position at that moment - the desired position. The 
motor controller then attempts to get the robot wheel to be at the desired position. If the motor controller 
is successful the wheel will be going at the speed specified in the cmd_vel message. 

This scheme was chosen for Magni, because velocity commands alwasy require the measurement of 2
positions in time and computing the difference between them. Because of this speed tends to be much
less accurate on a percentage basis than position, particularly when measuring frequently when there
probably isn't much position change. As such we found that we achieved significantly higher performance
with a position oriented controller than we did with a speed oriented controller, because the fundamental
basis for making decisions (position) is much more accurate than the alternative (velocity).

In the Magni Controller the moment a new motor control message is received, it computes a new target position based on
the current desired position (the theoretical position that the wheel is supposed to be in at that moment in time).
As such the target position in any moment in time is determined by all previous motor-controller messages
and when they were sent.

#Motor Control Messages from the Command Line

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

#Motor Control Messages from Python Code

Speed commands can also be sent to the motor via the `/cmd_vel` topic from Python
code. Insert the following code in to the header of your Python program.

```
[INSERT EXAMPLE PYTHON CODE HERE]
```
then insert the following example code in the body of your program.

```
[INSERT EXAMPLE PYTHON CODE HERE]
```

#Motor Control Messages from C++ Code

Speed commands can also be sent to the motor via the `/cmd_vel` topic from Python
code. Insert the following code in to the header of your Python program.

```
[INSERT EXAMPLE PYTHON CODE HERE]
```
then insert the following example code in the body of your program.

```
[INSERT EXAMPLE PYTHON CODE HERE]
```

##Sending Messages over ethernet.

As with all ROS messages they may be transmitted over a common network. For more information on how to do this please see:

[Insert link to a tutorial on using networking].

