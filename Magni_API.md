
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

ROS provides a number of [client libraries](http://wiki.ros.org/Client%20Libraries),
the most commonly used of which are C++ and Python.

There are a large number of [tutorials](http://wiki.ros.org/ROS/Tutorials)
available, that cover installing ROS, using it, and programming new
functionality.

## Motor Control Messages

Velocity commands can be sent to the robot via the `/cmd_vel` topic which is of
type [`Twist`](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html).

Magni implements a _dead-man_ timer, so that if `/cmd_vel` messages cease to be received, then the
robot stops. This ensures that the robot will return to a safe state if a process dies or an electrical
connection fails. The dead-man timer is set to 100ms.

Magni's motor control is position oriented - in that it tries to achieve the position (not the speed) that 
it expects the process wanted. When a `/cmd_vel` message is received by the motor controller, it 
assumes that the intent of the program is to get to a target position by the time the dead-man timer 
expires. The target position is the position that would be achieved if the motor-controller went at
exactly the cmd_vel message speed for the time it takes the motor controller went at the cmd_vel 
speed. The motor controller computes this position for each wheel 10,000 times per second.
If the motor controller is successful, the wheel will be going at the speed specified in the `/cmd_vel`]
message. 

This scheme was chosen for Magni, because velocity commands alwasy require the measurement of 2
positions in time and computing the difference between them. Because of this speed tends to be much
less accurate on a percentage basis than position, particularly when measuring frequently when there
probably isn't much position change. As such we found that we achieved significantly higher performance
with a position oriented controller than we did with a speed oriented controller, because the fundamental
basis for making decisions (position) is much more accurate than the alternative (velocity).

When a new motor control message is received, the motor controller calculates a new target position for
each wheel, based on the current desired position (the theoretical position that the wheel is supposed
to be in at that moment in time). The target position at any point in time is therefore determined by all
previous motor-controller messages and the time at which they were sent.

### Sending Motor Control Messages from the Command Line

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

### Sending Motor Control Messages from Python 

An example of how to send messages from Python is below.

```
import rospy
from geometry_msgs.msg import Twist

...

# Create a publisher
  cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

...

  # Create a message and publish it
  twist = Twist()
  twist.angular.z = 0.5
  twist.linear.x = 1.0
  cmd_pub.publish(twist)
```

For more information about sending ROS messages from Python, refer to the
[Writing a Simple Publisher and Subscriber (Python) tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29).

### Sending Motor Control Messages from C++ Code

An example of how to send messages from a C++ ROS node is below:

```
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

...

// Create a publisher
ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

...

   geometry_msgs::Twist msg;
   msg.angular.z = 0.5;
   msg.linear.x = 1.0;

   cmdPub.publish(msg);

```

For more information about sending ROS messages from Python, refer to the
[Writing a Simple Publisher and Subscriber (C++) tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(c%2B%2B)).

### Sending Messages over a Network

For an explanation of how to send messages from one machine to another, please refer to
the [Multiple Machines tutorial](http://wiki.ros.org/ROS/Tutorials/MultipleMachines).
