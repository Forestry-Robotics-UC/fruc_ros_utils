#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# Written: August 2024
# License: MIT License
#
# Program: ROS Bag File Processing Script
# Purpose: Processes ROS bag files, applies various transformations,
#          and provides utilities for managing bag files.

import os
import sys
import random
import argparse
import pathlib
import cv2
import rosbag
import argcomplete
from tqdm import tqdm
from PyQt5.QtWidgets import QApplication
from cv_bridge import CvBridge
from sensor_msgs.msg import Imu, Image, NavSatFix
import numpy as np  # Import numpy for statistical functions
import argparse
from pyqt_gui_utils import SimplePyQtGUIKit
from custom_logging import setup_custom_logger
from navsat_tools import NavSatExporter

# Setup the custom logger and CvBridge
logger = setup_custom_logger()
bridge = CvBridge()

def calculate_bag_duration(bag_path: str) -> float:
    """
    Calculate the duration (in seconds) of a ROS bag file by finding the first and last timestamp.
    """
    try:
        with rosbag.Bag(bag_path, 'r') as bag:
            # Get the first and last message timestamps
            start_time = bag.get_start_time()
            end_time = bag.get_end_time()
            
            # Calculate the duration (end time - start time)
            duration = end_time - start_time
            return duration
    except Exception as e:
        logger.error(f"Error processing bag {bag_path}: {e}")
        return 0.0

def sum_bag_durations_in_folder(folder_path: str) -> float:
    """
    Sum the total duration of all bag files in the specified folder.
    """
    total_duration = 0.0
    bag_files = [f for f in os.listdir(folder_path) if f.endswith('.bag')]

    # Iterate through all bag files in the directory
    for bag_file in tqdm(bag_files, desc="Processing bags"):
        bag_path = os.path.join(folder_path, bag_file)
        bag_duration = calculate_bag_duration(bag_path)
        total_duration += bag_duration

    # Return the total duration in seconds
    logger.info(f'Total duration of the {len(bag_files)} bags in the folder is: {total_duration}') 

def remove_topic_from_bag(bag_in_path: str, bag_out_path: str, topics: list) -> None:
    try:
        with rosbag.Bag(bag_out_path, 'w') as out_bag, rosbag.Bag(bag_in_path, 'r') as in_bag:
            total_messages = in_bag.get_message_count()
            for topic, msg, t in tqdm(
                in_bag.read_messages(),
                total=total_messages,
                desc=f'Processing {os.path.basename(bag_in_path)}'
            ):
                if topic not in topics:
                    out_bag.write(topic, msg, t)
        logger.info(f"Finished processing {bag_in_path}, saved to {bag_out_path}")
    except Exception as e:
        logger.error(f"Error in remove_topic_from_bag: {e}", exc_info=True)


def change_frame_id(bag_in_path: str, bag_out_path: str, topic: str, new_frame_id: str) -> None:
    """
    Change the frame ID for messages on a specific topic in the input bag file and save to the output bag file.
    """
    try:
        with rosbag.Bag(bag_in_path, 'r') as in_bag, rosbag.Bag(bag_out_path, 'w') as out_bag:
            total_messages = in_bag.get_message_count(topic_filters=[topic])
            for topic_name, msg, t in tqdm(
                in_bag.read_messages(topics=[topic]),
                total=total_messages,
                desc=f'Processing {topic}'
            ):
                if hasattr(msg, 'header'):
                    logger.debug(f"Previous header: {msg.header.frame_id}")
                    msg.header.frame_id = new_frame_id
                    logger.debug(f"New header: {msg.header.frame_id}")
                out_bag.write(topic_name, msg, t)
        logger.info(f"Finished processing {bag_in_path}, saved to {bag_out_path}")
    except Exception as e:
        logger.error(f"Error in change_frame_id: {e}", exc_info=True)


def print_topic_sizes(bag_path: str) -> None:
    """
    Print cumulative serialized message sizes (in MB and GB) for each topic in the bag file.
    """
    try:
        topic_size_dict = {}
        with rosbag.Bag(bag_path, 'r') as bag:
            for topic, msg, _ in bag.read_messages(raw=True):
                # msg is a tuple where the second element holds the serialized data
                topic_size_dict[topic] = topic_size_dict.get(topic, 0) + len(msg[1])
        sorted_topics = sorted(topic_size_dict.items(), key=lambda x: x[1])
        logger.info("Topic sizes in ascending order:")
        for topic, size in sorted_topics:
            size_mb = round(size / 1e6, 2)
            size_gb = round(size / 1e9, 4)
            logger.info(f"{topic}: {size_mb} MB, {size_gb} GB")
    except Exception as e:
        logger.error(f"Error in print_topic_sizes: {e}", exc_info=True)


def ned_to_enu(imu_msg: Imu) -> Imu:
    """
    Convert IMU data from NED to ENU coordinate frames.
    """
    enu_msg = Imu()
    enu_msg.header = imu_msg.header

    # Swap X and Y and invert Z for angular velocity
    enu_msg.angular_velocity.x = imu_msg.angular_velocity.y
    enu_msg.angular_velocity.y = imu_msg.angular_velocity.x
    enu_msg.angular_velocity.z = -imu_msg.angular_velocity.z

    # Swap X and Y and invert Z for linear acceleration
    enu_msg.linear_acceleration.x = imu_msg.linear_acceleration.y
    enu_msg.linear_acceleration.y = imu_msg.linear_acceleration.x
    enu_msg.linear_acceleration.z = -imu_msg.linear_acceleration.z

    # Swap X and Y and invert Z for orientation
    enu_msg.orientation.x = imu_msg.orientation.y
    enu_msg.orientation.y = imu_msg.orientation.x
    enu_msg.orientation.z = -imu_msg.orientation.z
    enu_msg.orientation.w = imu_msg.orientation.w

    return enu_msg


def convert_imu_to_enu(bag_in_path: str, bag_out_path: str, imu_topic: str) -> None:
    """
    Convert IMU data from NED to ENU for the specified topic in the input bag file and write to the output bag.
    """
    try:
        with rosbag.Bag(bag_out_path, 'w') as out_bag, rosbag.Bag(bag_in_path, 'r') as in_bag:
            total_messages = in_bag.get_message_count()
            for topic, msg, t in tqdm(
                in_bag.read_messages(),
                total=total_messages,
                desc=f'Processing {os.path.basename(bag_in_path)}'
            ):
                if topic == imu_topic:
                    enu_msg = ned_to_enu(msg)
                    out_bag.write(topic, enu_msg, t)
                else:
                    out_bag.write(topic, msg, t)
        logger.info(f"Finished converting {imu_topic} from NED to ENU in {bag_in_path}, saved to {bag_out_path}")
    except Exception as e:
        logger.error(f"Error in convert_imu_to_enu: {e}", exc_info=True)


def process_all_bags_in_folder(folder_path: str, func, *args) -> None:
    """
    Process all ROS bag files in the given folder using the specified function.
    """
    try:
        bag_files = [f for f in os.listdir(folder_path) if f.endswith('.bag')]
        for bag_file in tqdm(bag_files, desc='Processing all bags'):
            input_bag_path = os.path.join(folder_path, bag_file)
            output_bag_path = os.path.join(folder_path, f"processed_{bag_file}")
            func(input_bag_path, output_bag_path, *args)
            logger.info(f"Processed {bag_file}")
    except Exception as e:
        logger.error(f"Error in process_all_bags_in_folder: {e}", exc_info=True)

def compute_focus_score(cv_image) -> float:
    """
    Compute the Tenengrad focus score for a given image.

    :param cv_image: Input image in OpenCV format.
    :return: Focus score (higher = sharper)
    """
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    gradient_magnitude = cv2.magnitude(grad_x, grad_y)
    return gradient_magnitude.mean()

def save_random_images_from_topic(
    folder_path: str,
    output_dir: str,
    topic: str,
    num_images: int = 20,
    seed: int = 42,
) -> None:
    """
    Save a specified number of random, non-blurry images from the given topic across
    multiple ROS bag files (searched recursively in folder_path). This function first
    builds a candidate list (storing only minimal metadata) for images that pass the
    blur test. Then, it randomly selects from that candidate list and re-reads the bag
    files to save the images.

    Memory usage is optimized by not storing full image data in the candidate list.

    :param folder_path: Directory containing ROS bag files (searched recursively).
    :param output_dir: Directory where images will be saved.
    :param topic: ROS topic from which to extract images.
    :param num_images: Number of non-blurry images to save.
    :param seed: Random seed for reproducibility.
    :param blur_threshold: Threshold for the blur test.
    """
    min_threshold = 40
    threshold = 100
    logger.info(f"Folder: {folder_path}, Output: {output_dir}, Topic: {topic}")
    try:
        os.makedirs(output_dir, exist_ok=True)

        # Gather all bag file paths recursively.
        bag_files = [
            os.path.join(root, file)
            for root, _, files in os.walk(folder_path)
            for file in files if file.endswith('.bag')
        ]
        print(bag_files)
        if not bag_files:
            logger.warning(f"No bag files found in {folder_path}")
            return

        candidate_list = []  # List of tuples: (bag_file, timestamp)
        random.seed(seed)
        while theshold < min_threshold:
            # Pass 1: Process each bag file and collect candidate metadata only.
            for bag_file in bag_files:
                try:
                    with rosbag.Bag(bag_file, 'r') as bag:
                        for topic_name, msg, timestamp in bag.read_messages(topics=[topic]):
                            # Convert message to an image and check if it is not blurry.
                            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                            if not is_image_blurry(cv_image, threshold):
                                # Store minimal info; no heavy image data is kept.
                                candidate_list.append((bag_file, timestamp))
                except Exception as e:
                    logger.error(f"Error processing bag file {bag_file}: {e}", exc_info=True)

            total_candidates = len(candidate_list)
            if total_candidates < num_images*2:
                threshold -= 1
            if total_candidates == num_images*2 or threshold == min_threshold:
                logger.info(f"Number of images found: {total_candidates}")
                break

        logger.info(f"Found {total_candidates} candidate non-blurry images for topic {topic}")

        # Determine how many candidates to use (if there are fewer than requested, take all).
        num_to_select = min(num_images, total_candidates)
        selected_candidates = random.sample(candidate_list, num_to_select)

        saved_count = 0
        # Pass 2: For each selected candidate, re-read its bag file to retrieve and save the image.
        for bag_file, timestamp in selected_candidates:
            try:
                with rosbag.Bag(bag_file, 'r') as bag:
                    found = False
                    for topic_name, msg, t in bag.read_messages(topics=[topic]):
                        # Compare timestamps (assuming an exact match).
                        if t == timestamp:
                            dataset = pathlib.PurePath(bag_file)
                            dataset = dataset.parent.name
                            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                            image_filename = os.path.join(output_dir, f"rgb_{dataset}_{saved_count:04d}_{timestamp}.png")
                            cv2.imwrite(image_filename, cv_image)
                            # logger.info(f"Saved image {image_filename} from bag {bag_file}")
                            saved_count += 1
                            found = True
                            break
                    if not found:
                        logger.warning(f"Could not locate message with timestamp {timestamp} in {bag_file}")
            except Exception as e:
                logger.error(f"Error reprocessing bag file {bag_file}: {e}", exc_info=True)
        
        logger.info(f"Saved {saved_count}/{num_images} images to {output_dir}")
    except Exception as e:
        logger.error(f"Error in save_random_images_from_topic: {e}", exc_info=True)




def collect_parameters(params: dict, args: argparse.Namespace) -> None:
    """
    Map collected GUI parameters to the argparse namespace.
    """
    mapping = {
        'input_path': 'bagin',
        'output_path': 'bagout',
        'folder_path': 'folder_path',
        'topics': 'topic',
        'new_frame_id': 'new_frame_id',
        'num_images': 'num_images',
        'seed': 'seed'
    }

    for key, arg_name in mapping.items():
        if key in params:
            setattr(args, arg_name, params[key])

    # For save_random_images, we expect the output directory in 'output_dir'
    if 'output_path' in params:
        setattr(args, 'output_dir', params['output_path'])

    # Handle folder or file-specific logic
    input_path = params.get('input_path')
    if input_path and os.path.isdir(input_path):
        args.folder_path = input_path
        logger.info("Input is a folder.")
    else:
        args.bagin = input_path


def print_image_sizes(bag_path: str):
    """
    Print the size (height x width) of images for all image topics in the bag file.
    """
    try:
        logger.info(f"Processing {bag_path} to get image sizes...")

        # Initialize CvBridge to convert ROS Image messages to OpenCV images
        bridge = CvBridge()

        # Dictionary to store image topic names and their respective sizes
        image_sizes = {}

        with rosbag.Bag(bag_path, 'r') as bag:
            total_messages = bag.get_message_count()

            # Store already processed topics
            processed_topics = set()

            # Iterate through all messages in the bag file
            for topic, msg, _ in bag.read_messages():
                if topic not in processed_topics:  # Process each topic only once
                    processed_topics.add(topic)
                    if "Image" in msg._type:
                        try:
                            # print(f"Found image topic: {topic}")

                            # Convert the ROS Image message to a CV2 image
                            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

                            # Get the size of the image (height x width)
                            image_size = cv_image.shape[:2]  # Get the height and width (ignoring channels)
                            image_size = (image_size[1], image_size[0])  # Switch to (width, height) format
                            # print(image_size)
                            # Store the image size by topic
                            if topic not in image_sizes:
                                image_sizes[topic] = []
                            image_sizes[topic].append(image_size)

                        except Exception as e:
                            logger.warning(f"Failed to process image message from topic {topic}: {e}")

        # Output the sizes of the images for each topic
        log_message = "Image sizes by topic:\n"
        for topic, sizes in image_sizes.items():
            log_message += f"Topic: {topic}\n"
            for size in sizes:
                log_message += f"  Image size: {size[0]}x{size[1]}\n"

        log_message += f"Finished processing {bag_path}"

        # Log all in a single message to avoid multiple timestamps
        logger.info(log_message)

    except Exception as e:
        logger.error(f"Error in print_image_sizes: {e}", exc_info=True)

def filter_camera_topics(bag_in_path: str, bag_out_path: str) -> None:
    """
    Copy only camera-related topics (Image and CameraInfo) from the input bag to the output bag.

    A topic is considered camera-related if:
      - Its message type contains 'sensor_msgs/Image' or 'sensor_msgs/CompressedImage'
      - Or its name ends with '/camera_info'
    """
    try:
        with rosbag.Bag(bag_in_path, 'r') as in_bag, rosbag.Bag(bag_out_path, 'w') as out_bag:
            info = in_bag.get_type_and_topic_info().topics
            camera_topics = [
                t for t, v in info.items()
                if 'sensor_msgs/Image' in v.msg_type
                or 'sensor_msgs/CompressedImage' in v.msg_type
                or t.endswith('/camera_info')
            ]
            if not camera_topics:
                logger.warning(f"No camera topics found in {bag_in_path}")
                return

            total_messages = in_bag.get_message_count(topic_filters=camera_topics)
            for topic, msg, t in tqdm(
                in_bag.read_messages(topics=camera_topics),
                total=total_messages,
                desc=f'Filtering cameras from {os.path.basename(bag_in_path)}'
            ):
                out_bag.write(topic, msg, t)
        logger.info(f"Saved only camera topics from {bag_in_path} to {bag_out_path}")
    except Exception as e:
        logger.error(f"Error in filter_camera_topics: {e}", exc_info=True)

def main() -> None:
    """
    Main entry point for the ROS bag processing script.
    """
    parser = argparse.ArgumentParser(
        description=(
            'Process ROS bag files with various utilities. '
            'Functions include removing topics, changing frame IDs, converting IMU data from NED to ENU, merging bags, '
            'and printing topic sizes.'
        )
    )
    function_choices = [
        'remove_topic',
        'change_frame_id',
        'print_topic_sizes',
        'convert_imu_to_enu',
        'merge_bags',
        'save_random_images',
        'print_image_sizes', 
        'sum_bag_durations',
        'check_gps_quality',
        'export_navsatfix',
        'filter_camera_topics'
    ]
    if '--gui' not in sys.argv:
        parser.add_argument(
            'function',
            type=str,
            choices=function_choices,
            help=f'Function to run {function_choices}'
        )

    parser.add_argument('--bagin', type=str, help='Input bag file path.')
    parser.add_argument('--bagout', type=str, help='Output bag file path.')
    parser.add_argument('--topic', type=str, nargs='+', help='Topics to process.')
    parser.add_argument('--new_frame_id', type=str, help='New frame ID for the specified topic.')
    parser.add_argument('--folder_path', type=str, help='Path to the folder containing ROS bag files.')
    parser.add_argument('--gui', action='store_true', help='Use GUI for input selection.')
    parser.add_argument('--num_images', type=int, help='Number of random images to save.')
    parser.add_argument('--output_dir', type=str, help='Directory to save images.')
    parser.add_argument('--seed', type=int, help='Random seed for image selection.')
    parser.add_argument('--csv_out', type=str, help='Output CSV path for NavSatFix export.')

    argcomplete.autocomplete(parser)
    args = parser.parse_args()

    if args.gui:
        gui = SimplePyQtGUIKit()
        try:
            params = gui.PromptForParameters(function_choices)
            collect_parameters(params, args)
            args.function = params['function']
        except Exception as e:
            logger.error(f"Error during GUI input: {e}", exc_info=True)
            return

        logger.info(f"Arguments from GUI: {args}")
    else:
        if not hasattr(args, 'function') or not args.function:
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

    # Map function names to the corresponding processing routines.
    func_map = {
        'remove_topic': lambda: (
            process_all_bags_in_folder(args.folder_path, remove_topic_from_bag, args.topic)
            if args.folder_path else remove_topic_from_bag(args.bagin, args.bagout, args.topic)
        ),
        'change_frame_id': lambda: (
            process_all_bags_in_folder(args.folder_path, change_frame_id, args.topic[0], args.new_frame_id)
            if args.folder_path else change_frame_id(args.bagin, args.bagout, args.topic[0], args.new_frame_id)
        ),
        'print_topic_sizes': lambda: (
            process_all_bags_in_folder(args.folder_path, print_topic_sizes)
            if args.folder_path else print_topic_sizes(args.bagin)
        ),
        'convert_imu_to_enu': lambda: (
            process_all_bags_in_folder(args.folder_path, convert_imu_to_enu, args.topic[0])
            if args.folder_path else convert_imu_to_enu(args.bagin, args.bagout, args.topic[0])
        ),
        'merge_bags': lambda: merge_bags(args.folder_path, args.bagout),  # merge_bags should be defined elsewhere
        'save_random_images': lambda: save_random_images_from_topic(
            args.folder_path, args.output_dir, args.topic[0], args.num_images, args.seed
        ),
        'print_image_sizes': lambda: print_image_sizes(args.bagin),
        'print_image_sizes': lambda: print_image_sizes(args.bagin),
        'sum_bag_durations': lambda: sum_bag_durations_in_folder(args.folder_path) ,
        'check_gps_quality': lambda: print(
            NavSatExporter(
                topic=args.topic[0] if args.topic else '/fix'
            ).quality_report(
                folder=args.folder_path,
                csv_path=args.csv_out,
                recursive=True
            )
        ),

        'export_navsatfix': lambda: NavSatExporter(
            topic=args.topic[0] if args.topic else '/fix'
        ).export_csv(
            folder=args.folder_path,
            out_path=(args.csv_out or 'navsatfix_export.csv'),
            recursive=True
        ),
        'filter_camera_topics': lambda: (
    process_all_bags_in_folder(args.folder_path, filter_camera_topics)
    if args.folder_path else filter_camera_topics(args.bagin, args.bagout)
),
          }

    try:
        if args.function in func_map:
            func_map[args.function]()
        else:
            logger.error("Invalid function selected. Use --help for usage details.")
    except Exception as e:
        logger.error(f"Error during function execution: {e}", exc_info=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger.info("Script interrupted by user.")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Unexpected error: {e}", exc_info=True)
        sys.exit(1)
