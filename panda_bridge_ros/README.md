# panda_bridge_ros

Package to expose CAN frames using the awesome [panda board from comma.ai](https://github.com/commaai/panda) in ROS (in `can_msgs/Frame` format.)

Developed to expose the Honda Civic 2016 information in the [moveit hackathon](https://www.pixmoving.com/move-it) in Guiyang, China (March 2018).

# Dependences

**Note**: rosdep should resolve them once [this PR](https://github.com/ros/rosdistro/pull/17106) is accepted to add them as rosdep keys.

Python:

* [cantools](https://pypi.python.org/pypi/cantools/5.2.0): `pip install cantools`
* [pandacan](https://github.com/commaai/panda): `pip install pandacan`

ROS:
* [can_msgs](http://wiki.ros.org/can_msgs): `sudo apt-get install ros-DISTRO-can-msgs`


# Nodes

* [panda_bridge_ros.py](scripts/panda_bridge_ros.py): Contains the node that connects to the panda board and exposes in the `can_frame_msgs` `can_msgs/Frame` topic the messages read.
* [frame_decoder.py](scripts/frame_decoder.py): Node that subscribes to the `can_frame_msgs` topic and publishes in `can_frame_msgs_human_friendly` `std_msgs/String` a dictionary with the CAN bus message translated from the dbc file for Honda Civic 2016.


