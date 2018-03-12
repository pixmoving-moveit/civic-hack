#!/usr/bin/env python
import rosbag
from rospkg import RosPack


if __name__ == '__main__':
    rp = RosPack()
    path = rp.get_path('panda_bridge_ros')
    path_new_rosbag = path + '/data/brake_filtered.bag'
    path_rosbag = path + '/data/test_brake_2018-03-11-23-54-28.bag'

    count_msgs = 0
    with rosbag.Bag(path_new_rosbag, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(path_rosbag).read_messages():
            if topic == "/can_frame_msgs":
                # BRAKE_COMMAND   VSA_STATUS   BRAKE_PRESSURE
                if msg.id == 506 or msg.id == 420 or msg.id == 487:
                    outbag.write(topic, msg, t)
                    count_msgs += 1

    print("Wrote: " + str(count_msgs) + " messages")
