#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# Written: August 2024
# License: This code is licensed under the MIT License.
#
# Program: ROS Bag File Processing Script
# Purpose: Processes ROS bag files, applies various transformations, and provides utilities for managing bag files.

import os
import sys
import rosbag
import argparse
import argcomplete
from tqdm import tqdm
from PyQt5.QtWidgets import QApplication
from pyqt_gui_utils import SimplePyQtGUIKit
from cv_bridge import CvBridge
from sensor_msgs.msg import Imu
from custom_logging import setup_custom_logger

# Setup the custom logger
logger = setup_custom_logger()

# Initialize the CvBridge
bridge = CvBridge()

def remove_topic_from_bag(bag_in, bag_out, topics):
    """
    Remove specified topics from the input bag file and save to the output bag file.
    """
    try:
        with rosbag.Bag(bag_out, 'w') as outbag:
            with rosbag.Bag(bag_in) as inbag:
                total_messages = inbag.get_message_count()
                for topic, msg, t in tqdm(inbag.read_messages(), total=total_messages, desc=f'Processing {os.path.basename(bag_in)}'):
                    if topic not in topics:
                        outbag.write(topic, msg, t)
        logger.info(f"Finished processing {bag_in}, saved to {bag_out}")
    except Exception as e:
        logger.error(f"Error in remove_topic_from_bag: {e}", exc_info=True)

def change_frame_id(bag_in, bag_out, topic, new_frame_id):
    """
    Change the frame ID of the specified topic in the input bag file and save to the output bag file.
    """
    try:
        with rosbag.Bag(bag_in) as bag_in, rosbag.Bag(bag_out, 'w') as bag_out:
            total_messages = bag_in.get_message_count(topic_filters=[topic])
            for topic_name, msg, t in tqdm(bag_in.read_messages(topics=[topic]), total=total_messages, desc=f'Processing {topic}'):
                if topic_name == topic:
                    logger.debug(f"Previous header: {msg.header.frame_id}")
                    msg.header.frame_id = new_frame_id
                    logger.debug(f"New header: {msg.header.frame_id}")
                    bag_out.write(topic_name, msg, t)
                else:
                    bag_out.write(topic_name, msg, t)
        logger.info(f"Finished processing {bag_in}, saved to {bag_out}")
    except Exception as e:
        logger.error(f"Error in change_frame_id: {e}", exc_info=True)

def print_topic_sizes(bag_path):
    """
    Print total cumulative serialized message size per topic for a given bag file.
    """
    try:
        topic_size_dict = {}
        with rosbag.Bag(bag_path, 'r') as bag:
            for topic, msg, t in bag.read_messages(raw=True):
                topic_size_dict[topic] = topic_size_dict.get(topic, 0) + len(msg[1])
        
        topic_size = sorted(topic_size_dict.items(), key=lambda x: x[1])
        logger.info("Topic Size by Ascending Order:")
        
        for topic, size in topic_size:
            size_mb = round((10**-6) * size, 2)
            size_gb = round((10**-9) * size, 4)
            logger.info(f"{topic}: {size_mb} MB, {size_gb} GB")
    except Exception as e:
        logger.error(f"Error in print_topic_sizes: {e}", exc_info=True)

def ned_to_enu(imu_msg):
    """
    Convert IMU data from NED to ENU orientation and axes.
    """
    enu_imu_msg = Imu()
    enu_imu_msg.header = imu_msg.header

    # Convert angular velocity from NED to ENU: swap X/Y, invert Z
    enu_imu_msg.angular_velocity.x = imu_msg.angular_velocity.y
    enu_imu_msg.angular_velocity.y = imu_msg.angular_velocity.x
    enu_imu_msg.angular_velocity.z = -imu_msg.angular_velocity.z

    # Convert linear acceleration from NED to ENU: swap X/Y, invert Z
    enu_imu_msg.linear_acceleration.x = imu_msg.linear_acceleration.y
    enu_imu_msg.linear_acceleration.y = imu_msg.linear_acceleration.x
    enu_imu_msg.linear_acceleration.z = -imu_msg.linear_acceleration.z

    # Convert orientation from NED to ENU: swap X/Y, invert Z
    enu_imu_msg.orientation.x = imu_msg.orientation.y
    enu_imu_msg.orientation.y = imu_msg.orientation.x
    enu_imu_msg.orientation.z = -imu_msg.orientation.z
    enu_imu_msg.orientation.w = imu_msg.orientation.w

    return enu_imu_msg

def convert_imu_to_enu(bag_in, bag_out, imu_topic):
    """
    Convert IMU data from NED to ENU for a specified topic in the input bag file and save to the output bag file.
    """
    try:
        with rosbag.Bag(bag_out, 'w') as outbag:
            with rosbag.Bag(bag_in) as inbag:
                total_messages = inbag.get_message_count()
                for topic, msg, t in tqdm(inbag.read_messages(), total=total_messages, desc=f'Processing {os.path.basename(bag_in)}'):
                    if topic == imu_topic:
                        enu_imu_msg = ned_to_enu(msg)
                        outbag.write(topic, enu_imu_msg, t)
                    else:
                        outbag.write(topic, msg, t)
        logger.info(f"Finished converting {imu_topic} from NED to ENU in {bag_in}, saved to {bag_out}")
    except Exception as e:
        logger.error(f"Error in convert_imu_to_enu: {e}", exc_info=True)

def process_all_bags_in_folder(folder_path, function, *args):
    """
    Process all bag files in a folder using the specified function.
    """
    try:
        bag_files = [f for f in os.listdir(folder_path) if f.endswith('.bag')]
        for file_name in tqdm(bag_files, desc='Processing all bags'):
            input_bag_path = os.path.join(folder_path, file_name)
            output_bag_path = os.path.join(folder_path, f"processed_{file_name}")
            function(input_bag_path, output_bag_path, *args)
            logger.info(f"Processed {file_name}")
    except Exception as e:
        logger.error(f"Error in process_all_bags_in_folder: {e}", exc_info=True)

def merge_bags(folder_path, output_bag):
    """
    Merge all ROS bag files in the specified folder into a single output bag file.

    :param folder_path: Path to the folder containing ROS bag files.
    :param output_bag: Path to the output merged bag file.
    """
    try:
        bag_files = [f for f in os.listdir(folder_path) if f.endswith('.bag')]
        with rosbag.Bag(output_bag, 'w') as outbag:
            for file_name in tqdm(bag_files, desc='Merging bags'):
                input_bag_path = os.path.join(folder_path, file_name)
                with rosbag.Bag(input_bag_path, 'r') as inbag:
                    for topic, msg, t in inbag.read_messages():
                        outbag.write(topic, msg, t)
        logger.info(f"Successfully merged all bags into {output_bag}")
    except Exception as e:
        logger.error(f"Error in merge_bags: {e}", exc_info=True)

def main():
    parser = argparse.ArgumentParser(
        description=(
            'Process ROS bag files with various utilities. '
            'Functions include removing topics, changing frame IDs, converting IMU data from NED to ENU, merging bags, and printing topic sizes.'
        )
    )
    parser.add_argument('function', type=str, choices=[
        'remove_topic', 'change_frame_id', 'print_topic_sizes', 'convert_imu_to_enu', 'merge_bags'
    ], help='Function to run: remove_topic, change_frame_id, print_topic_sizes, convert_imu_to_enu, merge_bags', nargs='?' if '--use_gui' in sys.argv else None)

    parser.add_argument('--bagin', type=str, help='Input bag file path.')
    parser.add_argument('--bagout', type=str, help='Output bag file path.')
    parser.add_argument('--topic', type=str, nargs='+', help='Topics to process.')
    parser.add_argument('--new_frame_id', type=str, help='New frame ID for the specified topic.')
    parser.add_argument('--folder_path', type=str, help='Path to the folder containing ROS bag files.')
    parser.add_argument('--use_gui', action='store_true', help='Use GUI for input selection.')
    argcomplete.autocomplete(parser)
    args = parser.parse_args()

    if args.use_gui:
        gui = SimplePyQtGUIKit()
        try:
            params = gui.PromptForParameters()
        except Exception as e:
            logger.error(f"Error during GUI input: {e}", exc_info=True)
            return
        args.function = params['function']
        input_path = params['input_path']
        if os.path.isdir(input_path):
            args.folder_path = input_path
            logger.info("ITS A FOLDER")
        else:
            args.bagin = input_path
            if 'output_path' in params:
                args.bagout = params['output_path']

        if 'topics' in params:
            args.topic = params['topics']
        if 'new_frame_id' in params:
            args.new_frame_id = params['new_frame_id']
        logger.info(f"All arguments from GUI: {args}")

    else: 
        if not args.function:
            logger.error("Error: The 'function' argument is required when not using the GUI.")
            parser.print_help()
            sys.exit(1)

    # Validate required arguments based on the function
    if args.function in ['remove_topic', 'change_frame_id', 'convert_imu_to_enu'] and not args.topic:
        logger.error("Error: --topic argument is required for the selected function.")
        parser.print_help()
        sys.exit(1)

    if args.function == 'change_frame_id' and not args.new_frame_id:
        logger.error("Error: --new_frame_id argument is required for the change_frame_id function.")
        parser.print_help()
        sys.exit(1)

    if args.function in ['convert_imu_to_enu', 'change_frame_id', 'remove_topic'] and not (args.folder_path or args.bagin):
        logger.error("Error: Either --folder_path or --bagin argument is required for the selected function.")
        parser.print_help()
        sys.exit(1)

    func_map = {
        'remove_topic': lambda: process_all_bags_in_folder(args.folder_path, remove_topic_from_bag, args.topic) if args.folder_path else remove_topic_from_bag(args.bagin, args.bagout, args.topic),
        'change_frame_id': lambda: process_all_bags_in_folder(args.folder_path, change_frame_id, args.topic[0], args.new_frame_id) if args.folder_path else change_frame_id(args.bagin, args.bagout, args.topic[0], args.new_frame_id),
        'print_topic_sizes': lambda: process_all_bags_in_folder(args.folder_path, print_topic_sizes) if args.folder_path else print_topic_sizes(args.bagin),
        'convert_imu_to_enu': lambda: process_all_bags_in_folder(args.folder_path, convert_imu_to_enu, args.topic[0]) if args.folder_path else convert_imu_to_enu(args.bagin, args.bagout, args.topic[0]),
        'merge_bags': lambda: merge_bags(args.folder_path, args.bagout)
    }

    if args.function in func_map:
        try:
            func_map[args.function]()
        except Exception as e:
            logger.error(f"Error during function execution: {e}", exc_info=True)
    else:
        logger.error("Invalid arguments. Use --help for usage details.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger.info("Script interrupted by user.")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Unexpected error: {e}", exc_info=True)
        sys.exit(1)
