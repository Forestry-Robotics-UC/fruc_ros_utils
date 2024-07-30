#!/usr/bin/env python
import os
import sys
import rosbag
import argparse
import argcomplete
from tqdm import tqdm
import logging
from PyQt5.QtWidgets import QApplication
from pyqt_gui_utils import SimplePyQtGUIKit
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from scipy.spatial.transform import Rotation

# Initialize the logger
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

bridge = CvBridge()

def remove_topic_from_bag(bagin, bagout, topics):
    """
    Remove specified topics from the input bag file and save to the output bag file.

    :param bagin: Input bag file path.
    :param bagout: Output bag file path.
    :param topics: List of topics to remove.
    """
    try:
        with rosbag.Bag(bagout, 'w') as outbag:
            with rosbag.Bag(bagin) as inbag:
                total_messages = inbag.get_message_count()
                for topic, msg, t in tqdm(inbag.read_messages(), total=total_messages, desc=f'Processing {os.path.basename(bagin)}'):
                    if topic not in topics:
                        outbag.write(topic, msg, t)
        logging.info(f"Finished processing {bagin}, saved to {bagout}")
    except Exception as e:
        logging.error(f"Error in remove_topic_from_bag: {e}")

def change_frame_id(bagin, bagout, topic, new_frame_id):
    """
    Change the frame ID of the specified topic in the input bag file and save to the output bag file.

    :param bagin: Input bag file path.
    :param bagout: Output bag file path.
    :param topic: Topic to process.
    :param new_frame_id: New frame ID to set.
    """
    try:
        with rosbag.Bag(bagin) as bagIn, rosbag.Bag(bagout, 'w') as bagOut:
            total_messages = bagIn.get_message_count(topic_filters=[topic])
            for topic_name, msg, t in tqdm(bagIn.read_messages(topics=[topic]), total=total_messages, desc=f'Processing {topic}'):
                if topic_name == topic:
                    logging.debug(f"Previous header: {msg.header.frame_id}")
                    msg.header.frame_id = new_frame_id
                    logging.debug(f"New header: {msg.header.frame_id}")
                    bagOut.write(topic_name, msg, t)
                else:
                    bagOut.write(topic_name, msg, t)
        logging.info(f"Finished processing {bagin}, saved to {bagout}")
    except Exception as e:
        logging.error(f"Error in change_frame_id: {e}")

def print_topic_sizes(bag_path):
    """
    Print total cumulative serialized message size per topic for a given bag file.

    :param bag_path: Input bag file path.
    """
    try:
        topic_size_dict = {}
        with rosbag.Bag(bag_path, 'r') as bag:
            for topic, msg, t in bag.read_messages(raw=True):
                topic_size_dict[topic] = topic_size_dict.get(topic, 0) + len(msg[1])
        
        topic_size = sorted(topic_size_dict.items(), key=lambda x: x[1])
        logging.info("Topic Size by Ascending Order:")
        
        for topic, size in topic_size:
            size_mb = round((10**-6) * size, 2)
            size_gb = round((10**-9) * size, 4)
            logging.info(f"{topic}: {size_mb} MB, {size_gb} GB")
    except Exception as e:
        logging.error(f"Error in print_topic_sizes: {e}")

def process_all_bags_in_folder(folder_path, function, *args):
    """
    Process all bag files in a folder using the specified function.

    :param folder_path: Path to the folder containing ROS bag files.
    :param function: The function to apply to each bag file.
    :param args: Additional arguments to pass to the function.
    """
    try:
        bag_files = [f for f in os.listdir(folder_path) if f.endswith('.bag')]
        for file_name in tqdm(bag_files, desc='Processing all bags'):
            input_bag_path = os.path.join(folder_path, file_name)
            output_bag_path = os.path.join(folder_path, f"filtered_{file_name}")
            function(input_bag_path, output_bag_path, *args)
            logging.info(f"Processed {file_name}")
    except Exception as e:
        logging.error(f"Error in process_all_bags_in_folder: {e}")

def main():
    parser = argparse.ArgumentParser(description='Process ROS bag files.')
    parser.add_argument('function', type=str, choices=['remove_topic', 'change_frame_id', 'print_topic_sizes'],
                        help='Function to run: remove_topic, change_frame_id, print_topic_sizes')
    parser.add_argument('--bagin', type=str, help='Input bag file path.')
    parser.add_argument('--bagout', type=str, help='Output bag file path.')
    parser.add_argument('--topic', type=str, nargs='+', help='Topics to process.')
    parser.add_argument('--new_frame_id', type=str, help='New frame ID for the specified topic.')
    parser.add_argument('--folder_path', type=str, help='Path to the folder containing ROS bag files.')
    parser.add_argument('--use_gui', action='store_true', help='Use GUI for input selection.')
    argcomplete.autocomplete(parser)
    args = parser.parse_args()

    if args.use_gui:
        app = QApplication(sys.argv)
        gui = SimplePyQtGUIKit()
        try:
            params = gui.PromptForParameters(args.function)
        except Exception as e:
            logging.error(f"Error during GUI input: {e}")
            return

        input_path = params['input_path']
        if os.path.isdir(input_path):
            args.folder_path = input_path
        else:
            args.bagin = input_path
            if 'bagout' in params:
                args.bagout = params['bagout']

        if 'topics' in params:
            args.topic = params['topics']
        if 'new_frame_id' in params:
            args.new_frame_id = params['new_frame_id']

    func_map = {
        'remove_topic': lambda: process_all_bags_in_folder(args.folder_path, remove_topic_from_bag, args.topic) if args.folder_path else remove_topic_from_bag(args.bagin, args.bagout, args.topic),
        'change_frame_id': lambda: process_all_bags_in_folder(args.folder_path, change_frame_id, args.topic[0], args.new_frame_id) if args.folder_path else change_frame_id(args.bagin, args.bagout, args.topic[0], args.new_frame_id),
        'print_topic_sizes': lambda: process_all_bags_in_folder(args.folder_path, print_topic_sizes) if args.folder_path else print_topic_sizes(args.bagin)
    }

    if args.function in func_map:
        try:
            func_map[args.function]()
        except Exception as e:
            logging.error(f"Error during function execution: {e}")
    else:
        logging.error("Invalid arguments. Use --help for usage details.")

if __name__ == "__main__":
    main()
