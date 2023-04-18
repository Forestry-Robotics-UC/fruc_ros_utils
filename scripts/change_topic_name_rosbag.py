#!/usr/bin/env python3
import rosbag
from copy import deepcopy
import tf


bagIn = rosbag.Bag('/mnt/backup/rosbags/2019_11_15_ranger/2019_11_15_semfire.bag')
bagOut = rosbag.Bag('/mnt/backup/rosbags/2019_11_15_ranger/2019_11_15_semfire2.bag','w')

with bagOut as outbag:
    source_frame = 'dalsa_link'
    target_frame = 'dalsa_optical_frame'
    source_topic = '/dalsa_camera_720p/compressed'
    tf_topic = '/tf'
    new_topic = '/dalsa_camera_720p/compressed'

    for topic, msg, t in bagIn.read_messages():
        new_msg = deepcopy(msg)
        if topic == source_topic:
            if new_msg.header.frame_id == source_frame:
                new_msg.header.frame_id = target_frame
            outbag.write(new_topic, new_msg, t)
        else :
            outbag.write(topic, msg, t)

bagIn.close()
bagOut.close()
