# panda_bridge_ros

Package to expose CAN frames using the awesome [panda board from comma.ai](https://github.com/commaai/panda) in ROS (in `can_msgs/Frame` format.)

Developed to expose the Honda Civic 2016 information in the [moveit hackathon](https://www.pixmoving.com/move-it) in Guiyang, China (March 2018).

Note that you may need to follow the [panda instructions for the udev rules](https://community.comma.ai/wiki/index.php/Panda).

# Dependences

You can install the dependences running `rosdep install panda_bridge_ros` (if you already made a workspace with this package checked out). Or `rosdep install panda_bridge_ros --from-paths YOUR_PATH_TO_panda_bridge_ros`.

Python:

* [cantools](https://pypi.python.org/pypi/cantools/5.2.0): `pip install cantools`
* [pandacan](https://github.com/commaai/panda): `pip install pandacan`

ROS:
* [can_msgs](http://wiki.ros.org/can_msgs): `sudo apt-get install ros-DISTRO-can-msgs`


# Nodes

* [panda_bridge_ros.py](scripts/panda_bridge_ros.py): Contains the node that connects to the panda board and exposes in the `/can_frame_msgs` `can_msgs/Frame` topic the messages read.
* [bridge_and_send.py](scripts/bridge_and_send.py): Contains the node that connects to the panda board and exposes in the `/can_frame_msgs` `can_msgs/Frame` topic the messages read, it also listens to the topic `/send_can_msg` which will send the `can_msgs/Frame` published to the CAN bus.
* [frame_decoder.py](scripts/frame_decoder.py): Node that subscribes to the `can_frame_msgs` topic and publishes in `/can_frame_msgs_human_friendly` `std_msgs/String` a dictionary with the CAN bus message translated from the dbc file for Honda Civic 2016. Understood messages can be found in [Understood_messages.md](Understood_messages.md).


# Example outputs

```bash
rostopic echo /can_frame_msgs
header: 
  seq: 1352
  stamp: 
    secs: 1520746948
    nsecs: 469939947
  frame_id: ''
id: 330
is_rtr: False
is_extended: False
is_error: False
dlc: 8
data: [254, 239, 255, 161, 7, 254, 210, 3]
---
header: 
  seq: 1353
  stamp: 
    secs: 1520746948
    nsecs: 470077037
  frame_id: ''
id: 399
is_rtr: False
is_extended: False
is_error: False
dlc: 8
data: [227, 198, 252, 35, 48, 1, 9, 0]
```


```bash
rostopic echo /can_frame_msgs_human_friendly
data: {'STEER_ANGLE_OFFSET': -0.7000000000000001, 'CHECKSUM': 3, 'COUNTER': 3, 'STEER_ANGLE_RATE': 0, 'frame_id': 330, 'STEER_WHEEL_ANGLE': 25.6, 'raw_msg': '\xff\x00\x00\x00\x07\xff\x003', 'STEER_ANGLE': 25.6, 'message_name': 'STEERING_SENSORS'}
---
data: {'STEER_STATUS': 3, 'STEER_TORQUE_SENSOR': -89, 'message_name': 'STEER_STATUS', 'CHECKSUM': 10, 'STEER_CONTROL_ACTIVE': 0, 'COUNTER': 3, 'frame_id': 399, 'raw_msg': '\xff\xa7\x00\x000\x01:\x00', 'STEER_TORQUE_MOTOR': 0}
```

There is a [rosbag in the data folder](data/example_msgs_2018-03-11-16-42-27.bag) which you can play to test.
