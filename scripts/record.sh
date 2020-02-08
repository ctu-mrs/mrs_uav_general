#!/bin/bash

path="/home/\$(optenv USER mrs)/bag_files/latest/"

exclude=(

# IN GENERAL, DON'T RECORD CAMERAS
# if you want to record cameras, create a copy of this script
# and place it at you tmux session.

# Every compressed
'(.*)compressed(.*)'
# Every image raw
'(.*)image_raw(.*)'
# Every theora message
'(.*)theora(.*)'
# Every h264 message
'(.*)h264(.*)'

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
