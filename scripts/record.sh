#!/bin/bash

filename=`mktemp`

exclude=(
# mobius
'/$(arg UAV_NAME)/mobius/image_raw'
'/$(arg UAV_NAME)/mobius/image_raw/compressed/(.*)'
'/$(arg UAV_NAME)/mobius/image_raw/compressedDepth(.*)'
'/$(arg UAV_NAME)/mobius/image_raw/theora(.*)'
# bluefox
'/$(arg UAV_NAME)/bluefox/image_raw'
'/$(arg UAV_NAME)/bluefox/image_raw/compressed/(.*)'
'/$(arg UAV_NAME)/bluefox/image_raw/compressedDepth(.*)'
'/$(arg UAV_NAME)/bluefox/image_raw/theora(.*)'
# Realsense t265
'/$(arg UAV_NAME)/rs_t265/fisheye(.*)'
# Realsesne d435
'(.*)rs_d435(.*)depth_to_infra(.*)'
'(.*)rs_d435(.*)depth_to_color/image_raw'
'(.*)rs_d435(.*)depth_to_color(.*)compressed'
'(.*)rs_d435(.*)depth_to_color(.*)compressed/(.*)'
'(.*)rs_d435(.*)color/image_raw'
'(.*)rs_d435(.*)/depth/(.*)'
'(.*)rs_d435(.*)/infra(.*)'
'(.*)rs_d435(.*)/color/image_rect_color'
# Every theora message
'(.*)theora(.*)'
)

# file's header
echo "<launch>" > "$filename"
echo "<arg name=\"UAV_NAME\" default=\"\$(env UAV_NAME)\" />" >> "$filename"
echo "<group ns=\"\$(arg UAV_NAME)\">" >> "$filename"

echo -n "<node pkg=\"rosbag\" type=\"record\" name=\"rosbag_record\" args=\"-o /home/\$(optenv USER mrs)/bag_files/latest/ -a" >> "$filename"

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
