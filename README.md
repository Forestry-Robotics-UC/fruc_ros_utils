
# ROS Utils

This repository contains a collection of utility scripts for ROS (Robot Operating System). These scripts provide various functionalities, including image processing, USB device monitoring, ROS bag file manipulation, and system monitoring.

## Contents

1. [Image Subscriber Template](#image-subscriber-template)
2. [USB Buffer Publisher](#usb-buffer-publisher)
3. [Bag Utils](#bag-utils)
4. [System Monitoring](#system-monitoring)

## Image Subscriber Template

This script subscribes to a ROS image topic, converts the ROS image messages to OpenCV images, and publishes the processed images.

### Usage

1. **Run the script**:
    ```bash
    rosrun <your_package_name> image_subscriber_template.py
    ```

2. **Parameters**:
    - `~publish_rate` (int, default: 50): The rate at which to publish debug images.

3. **ROS Topics**:
    - Subscribes to: `~image_topic` (sensor_msgs/Image)
    - Publishes to: `~debug_image` (sensor_msgs/Image)

## USB Buffer Publisher

This script monitors USB devices, captures USB traffic, and retrieves device descriptors.

### Usage

1. **Run the script**:
    ```bash
    python usb_buffer_publisher.py
    ```

## Bag Utils

This script provides utilities for processing ROS bag files, including removing topics, changing frame IDs, and printing topic sizes. It also supports a GUI option for input selection.

### Usage

1. **Remove a topic from a single bag file**:
    ```bash
    python bagutils.py remove_topic --bagin input.bag --bagout output.bag --topic /topic_to_remove
    ```

2. **Change frame ID for a topic in a single bag file**:
    ```bash
    python bagutils.py change_frame_id --bagin input.bag --bagout output.bag --topic /topic_name --new_frame_id new_frame
    ```

3. **Print total cumulative serialized message size per topic for a single bag file**:
    ```bash
    python bagutils.py print_topic_sizes --bagin input.bag
    ```

4. **Process all bag files in a folder**:
    ```bash
    python bagutils.py --folder_path /path/to/bag/folder --function remove_topic --topic /topic_to_remove
    python bagutils.py --folder_path /path/to/bag/folder --function change_frame_id --topic /topic_name --new_frame_id new_frame
    python bagutils.py --folder_path /path/to/bag/folder --function print_topic_sizes
    ```

5. **Use GUI for input selection**:
    ```bash
    python bagutils.py --use_gui
    ```

## System Monitoring

This script monitors system metrics such as CPU temperatures, CPU frequencies, and ROS topic frequencies, publishing them as diagnostics messages.

### Usage

1. **Run the script**:
    ```bash
    rosrun <your_package_name> system_monitoring.py
    ```

2. **ROS Topics**:
    - Publishes to: `/diagnostics/system` (diagnostic_msgs/DiagnosticArray)
