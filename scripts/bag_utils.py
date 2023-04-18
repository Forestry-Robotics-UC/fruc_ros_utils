#!/usr/bin/env python
import rosbag
from copy import deepcopy
import tf
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np
from scipy.spatial.transform import Rotation
import os
import os.path
import xml.etree.ElementTree as ET
from xml.dom import minidom
bagin = '/home/duda/datasets/quintareifmd/navigator__2021-06-22-14-19-20_0_test.bag'
bagout ='/home/duda/datasets/quintareifmd/navigator__2021-06-22-14-19-20_0_test2.bag'

path = '/home/duda/datasets/quintareifmd/'

source_topic = ['/zed_nano/zed_node/left/image_rect_color/compressed',
                '/zed_nano/zed_node/depth/depth_registered/compressedDepth',
                '/tf_static',
                '/zed_nano/zed_node/depth/camera_info',
                '/zed_nano/zed_node/right/image_rect_color/compressed',
                '/zed_nano/zed_node/right/camera_info ']
bridge = CvBridge()


def get_bags_from_folder(path):
  bags = []
  file_list = os.listdir(path)

  for file in file_list:
    if 'bag' in file:
      bags.append(path+'/'+file.split('.')[0])
  return bags

def change_image_color(image_msg):
  pass #TODO


def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ET.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

def quat_to_rpy(tf_msg):
  rotation = [tf.transform.rotation.x,
              tf.transform.rotation.y,
              tf.transform.rotation.z,
              tf.transform.rotation.w]
  rot = Rotation.from_quat(rotation)
  rot_euler = rot.as_euler('xyz', degrees=True)
  tvec = np.asarray([tf.transform.translation.x,
                     tf.transform.translation.y,
                     tf.transform.translation.z])
  print(tf.header.frame_id, tf.child_frame_id, rot_euler, tvec)

  return tf.header.frame_id, tf.child_frame_id, rot_euler, tvec

def change_timestamp(topic, new_msg, t):
  if 'tf' in topic:
    print("Analyzing tf topic: ", topic)
    for tf in new_msg.transforms:
      tf.header.stamp = t
      # print("New tf time: ", tf.header.stamp)
    new_topic = new_msg
  else:
    new_topic = new_msg
    new_topic.header.stamp = t
    # print(topic, " Old time: ", new_msg.header.stamp)
    # print(topic, " New time: ", new_topic.header.stamp)
  return new_topic

def urdf_from_tf_static(bagin):
  #TODO
  tf_list = []
  bagIn = rosbag.Bag(bagin)
  for topic, msg, t in bagIn.read_messages(topics=['/tf_static']):
    tfs = get_tf_list(msg, tf_list)
  bagIn.close()


  # adding an element to the root node
  attrib = {'name': 'sensorbox'}
  root = ET.Element("Robot",attrib)
  print("number of tfs: ", len(tfs))
  for tf in tfs:
    print(tf)
    attrib['name'] = tf
    element = root.makeelement('link', attrib)
    root.append(element)
  # mydata = ET.tostring(root)
  mydata = prettify(root)
  print(type(mydata))
  myfile = open(path+"items2.xml", "wb")
  myfile.write(mydata)
  # # adding an element to the seconditem node
  # attrib = {'name2': 'secondname2'}
  # subelement = root[0][1].makeelement('seconditem', attrib)
  # ET.SubElement(root[1], 'seconditem', attrib)
  # root[1][0].text = 'seconditemabc'


def get_tf_list(new_msg, list_tf):
  for tf in new_msg.transforms:
    # print(tf)
    list_tf.append(tf.child_frame_id)
  return list_tf

def read_write_bag(bagin, bagout, src_topic):
  bagIn = rosbag.Bag(bagin)
  bagOut = rosbag.Bag(bagout, 'w')
  print("Reading bag: ", bagin)
  write=True
  with bagOut as outbag:
    for topic, msg, t in bagIn.read_messages():
      new_msg = deepcopy(msg)
      for topic_name in src_topic:
        if topic == topic_name:
          if 'tf' in topic:
            for tf in new_msg.transforms:
              tf.header.stamp = t
              # print("New tf time: ", tf.header.stamp)
            new_topic = new_msg
          else:
            new_topic = new_msg
            new_topic.header.stamp = t

          outbag.write(topic, new_topic, t)
          write=False
          break
      if write:
        outbag.write(topic, msg, t)
      else:
        write=True

  bagIn.close()
  bagOut.close()

def main():
  bags = get_bags_from_folder(path)
  print(bags)
  final_count = len(bags)
  init = 1
  for bag in bags:
    bag_write = bag + '_new_timestamssp.bag'
    read_write_bag(bag+'.bag', bag_write, source_topic)
    print("Finished bag ", init, 'of ', final_count)    
    print('*'*80)
    init += 1
    break

if __name__ == '__main__':
  main()
