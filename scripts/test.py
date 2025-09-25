import argcomplete
import argparse

parser = argparse.ArgumentParser(description="Debug Argcomplete")
parser.add_argument('function', choices=[
    'remove_topic', 'change_frame_id', 'print_topic_sizes', 
    'convert_imu_to_enu', 'merge_bags', 'save_random_images'
], help='Function to run')

print("Debug: Autocomplete Initialized")  # Debug output
argcomplete.autocomplete(parser)

if __name__ == "__main__":
    args = parser.parse_args()
    print(args)
