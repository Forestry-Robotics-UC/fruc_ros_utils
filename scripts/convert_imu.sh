#!/bin/bash

# Define the list of folders
folders=(
    "/media/duda/Elements/ENTTAC/data/site_Gascola/2024_08_02/collect_01/raw/"
    "/media/duda/Elements/ENTTAC/data/site_Gascola/2024_08_02/collect_02/raw/"
    "/media/duda/Elements/ENTTAC/data/site_FlagstaffHill/2024_07_18/collect_01/raw"
    "/media/duda/Elements/ENTTAC/data/site_FlagstaffHill/2024_07_18/collect_02/raw"
    "/media/duda/Elements/ENTTAC/data/site_FlagstaffHill/2024_07_19/collect_01/raw"
)

# Topic to be converted
topic="/vectornav/IMU"

# Loop through each folder and run the conversion
for folder in "${folders[@]}"; do
    echo "Processing folder: $folder"
    python bagutils.py convert_imu_to_enu --folder_path "$folder" --topic "$topic"
    if [ $? -eq 0 ]; then
        echo "Successfully processed $folder"
    else
        echo "Failed to process $folder" >&2
    fi
done
