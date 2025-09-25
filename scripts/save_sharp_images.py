import os
import time
import logging
import rosbag
import cv2
import numpy as np
from tqdm import tqdm
from cv_bridge import CvBridge
from sensor_msgs.msg import Imu
import random 
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

from custom_logging import setup_custom_logger

bridge = CvBridge()
logger = setup_custom_logger()


def compute_focus_score(cv_image) -> float:
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    gradient_magnitude = cv2.magnitude(grad_x, grad_y)
    return gradient_magnitude.mean()


def collect_focus_and_motion_data(bag_files, image_topic, imu_topic):
    focus_data = []  # (focus_score, motion_score, bag_file, timestamp)

    for bag_file in tqdm(bag_files, desc="Processing bag files"):
        imu_data = []

        with rosbag.Bag(bag_file, 'r') as bag:
            for topic, msg, t in bag.read_messages():
                if topic == imu_topic:
                    gyro = msg.angular_velocity
                    score = np.linalg.norm([gyro.x, gyro.y, gyro.z])
                    imu_data.append((t.to_sec(), score))

        imu_data.sort()
        imu_times = [t for t, _ in imu_data]
        imu_scores = [s for _, s in imu_data]

        def find_closest_motion(ts):
            if not imu_times:
                return 0.0
            diffs = [abs(ts - t) for t in imu_times]
            return imu_scores[np.argmin(diffs)]

        with rosbag.Bag(bag_file, 'r') as bag:
            total_msgs = bag.get_message_count(topic_filters=[image_topic])
            for topic, msg, t in tqdm(
                bag.read_messages(topics=[image_topic]),
                desc=f"Images: {os.path.basename(bag_file)}",
                total=total_msgs,
                leave=False
            ):
                try:
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    focus = compute_focus_score(cv_image)
                    motion = find_closest_motion(t.to_sec())
                    focus_data.append((focus, motion, bag_file, t))
                except Exception as e:
                    logger.warning(f"Error converting image from {bag_file}: {e}")

    return focus_data


def plot_focus_motion_scatter(focus_data):
    if not HAS_MATPLOTLIB:
        logger.warning("matplotlib not available — skipping plot.")
        return

    focus_scores = [x[0] for x in focus_data]
    motion_scores = [x[1] for x in focus_data]
    plt.figure(figsize=(10, 6))
    plt.scatter(focus_scores, motion_scores, alpha=0.5, s=10)
    plt.xlabel("Tenengrad Focus Score")
    plt.ylabel("IMU Motion Score (Angular Velocity Norm)")
    plt.title("Focus vs Motion Scatter Plot")
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def main(folder_path, image_topic="/camera/image_raw", imu_topic="/imu/data",
         target_count=20, debug=False, output_dir=None):
    start_time = time.perf_counter()

    bag_files = [
        os.path.join(root, file)
        for root, _, files in os.walk(folder_path)
        for file in files if file.endswith(".bag")
    ]

    if not bag_files:
        logger.warning("No bag files found.")
        return

    logger.info(f"Processing {len(bag_files)} bag files in {folder_path}")
    focus_data = collect_focus_and_motion_data(bag_files, image_topic, imu_topic)
    logger.info(f"Collected {len(focus_data)} image-motion pairs.")

    if debug:
        plot_focus_motion_scatter(focus_data)

    # Threshold sweep logic
    focus_threshold = 60
    motion_threshold = 0.8
    min_focus = 30
    min_motion = 0
    step = 0.5

    while focus_threshold >= min_focus and motion_threshold >= min_motion:
        filtered = [
            (bf, ts, f, m) for f, m, bf, ts in focus_data
            if f >= focus_threshold and m <= motion_threshold
        ]
        if debug:
            logger.info(f"Thresholds → Focus: {focus_threshold}, Motion: {motion_threshold} | Matches: {len(filtered)}")
        if len(filtered) >= 3 * target_count:
            logger.info(f"✅ Sufficient candidates found: {len(filtered)}")
            break

        focus_threshold -= step
        # motion_threshold -= 0.05

    logger.info(f"📊Candidates found: {len(filtered)}")
    logger.info(f"⏱️ Completed in {time.perf_counter() - start_time:.2f} seconds")
    # Determine how many candidates to use (if there are fewer than requested, take all).
    num_to_select = min(target_count, len(filtered))
    selected_candidates = random.sample(filtered, num_to_select)
    logger.info(f"📊Number of random candidates for target count: {len(selected_candidates)}")

    if output_dir:
        import pathlib
        os.makedirs(output_dir, exist_ok=True)
        logger.info(f"Saving selected images to {output_dir}")

        from cv_bridge import CvBridgeError
        saved_count = 0

        for bag_file, timestamp, *_ in tqdm(selected_candidates, desc="Saving images"):
            try:
                with rosbag.Bag(bag_file, 'r') as bag:
                    for topic_name, msg, t in bag.read_messages(topics=[image_topic]):
                        if t == timestamp:
                            try:
                                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                                dataset = pathlib.PurePath(bag_file)
                                dataset = dataset.parent.name
                                filename = f"rgb_{dataset}_{saved_count:04d}_{timestamp}.png"
                                path = os.path.join(output_dir, filename)
                                rotated = cv2.rotate(cv_image, cv2.ROTATE_180)
                                cv2.imwrite(path, rotated)
                                saved_count += 1
                            except CvBridgeError as e:
                                logger.warning(f"Failed to convert image: {e}")
                            break
            except Exception as e:
                logger.error(f"Failed to read bag {bag_file}: {e}", exc_info=True)

        logger.info(f"✅ Saved {saved_count}/{len(filtered)} images.")
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Debug focus/motion thresholds via scatter plot")
    parser.add_argument("folder", help="Folder containing bag files")
    parser.add_argument("--image_topic", default="/right/camera/image_raw")
    parser.add_argument("--imu_topic", default="/vectornav/IMU")
    parser.add_argument("--target_count", type=int, default=100, help="Target number of images to select")
    parser.add_argument("--debug", action="store_true", help="Enable scatter plot visualization")
    parser.add_argument("--output_dir", type=str, help="Directory to save selected images")
    args = parser.parse_args()

main(args.folder, args.image_topic, args.imu_topic, args.target_count, args.debug, args.output_dir)