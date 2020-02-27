#!/bin/bash

path="/home/\$(optenv USER mrs)/bag_files/latest/"

# By default, we record everything.
# Except for this list of EXCLUDED topics:
exclude=(

# IN GENERAL, DON'T RECORD CAMERAS
#
# If you want to record cameras, create a copy of this script
# and place it at you tmux session.
#
# Please, seek an advice of a senior researcher of MRS about
# what can be recorded. Recording too much data can lead to
# ROS communication hiccups, which can lead to eland, failsafe
# or just a CRASH.

# Image detect bluefox
'/mv(.*)/compressed(.*)'
# Bluefox
'(.*)bluefox_left/image_raw'
'(.*)bluefox_right/image_raw'
'(.*)bluefox_left/image_raw/h264'
'(.*)bluefox_right/image_raw/h264'
# Realsense d435
'(.*)/rs_d435/aligned_depth_to_color/(.*)compressed(.*)'
'(.*)/rs_d435/aligned_depth_to_infra(.*)'
'(.*)/rs_d435/color/image_raw'
'(.*)/rs_d435/color/image_rect_color'
'(.*)/rs_d435/depth(.*)'
'(.*)/rs_d435/infra(.*)'
# every compressedDepth
'(.*)compressedDepth(.*)'
# every theora
'(.*)/theora(.*)'
# ouster data
'(.*)/os1_cloud_node/points'
'(.*)/os1_cloud_node/points_processed'
'(.*)/os1_cloud_node/points_over_max_range'
'(.*)/os1_cloud_node/scan'
'(.*)/imu_packets'
'(.*)/lidar_packets'
# aloam
'(.*)/aloam_registration/(.*)'
'(.*)/aloam_odometry/(.*)cloud(.*)'
'(.*)/aloam_mapping/(.*)cloud(.*)'
'(.*)/aloam_mapping/aft_mapped_path'
# mapping
'(.*)/octomap_(.*)'
'(.*)/occupied_cells_vis_array'
'(.*)/free_cells_vis_array'
'(.*)/projected_map'
# general visualization
'(.*)/disturbances_markers'
)

# file's header
filename=`mktemp`
echo "<launch>" > "$filename"
echo "<arg name=\"UAV_NAME\" default=\"\$(env UAV_NAME)\" />" >> "$filename"
echo "<group ns=\"\$(arg UAV_NAME)\">" >> "$filename"

echo -n "<node pkg=\"rosbag\" type=\"record\" name=\"rosbag_record\" args=\"-o $path -a" >> "$filename"

# if there is anything to exclude
if [ "${#exclude[*]}" -gt 0 ]; then

  echo -n " -x " >> "$filename"

  # list all the string and separate the with |
  for ((i=0; i < ${#exclude[*]}; i++));
  do
    echo -n "${exclude[$i]}" >> "$filename"
    if [ "$i" -lt "$( expr ${#exclude[*]} - 1)" ]; then
      echo -n "|" >> "$filename"
    fi
  done

fi

echo "\" />" >> "$filename"

# file's footer
echo "</group>" >> "$filename"
echo "</launch>" >> "$filename"

cat $filename
roslaunch $filename
