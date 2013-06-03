#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 prefix_path"
    echo "  example: $0 /home/user/my_workspace"
    exit 1
fi

cmd_exists git || die 'git was not found'

prefix=$(cd $1 && pwd)

official="gencpp genlisp genmsg genpy message_generation message_runtime roscpp_core std_msgs"
for p in $official; do
    git clone https://github.com/ros/$p.git $prefix/$p
done

tully="ros_comm"
for p in $tully; do
    git clone https://github.com/tfoote/$p.git $prefix/$p
    cd $prefix/$p && git checkout android
done

cd $prefix/ros_comm
git apply $my_loc/files/ros_comm/*.patch

cd $prefix/roscpp_core && git checkout android

[ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'

cd $CMAKE_PREFIX_PATH
mkdir -p catkin_ws/src && cd catkin_ws/src
for p in $official; do
    ln -sf $prefix/$p ./
done

ln -sf $prefix/ros_comm/tools/rosbag_storage ./
ln -sf $prefix/ros_comm/test/test_rosbag_storage ./

exit 0
