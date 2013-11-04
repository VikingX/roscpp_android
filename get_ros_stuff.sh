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

mkdir -p $
 
[ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'
 
cd $CMAKE_PREFIX_PATH
mkdir -p catkin_ws/src && cd catkin_ws

wstool init -j8 src $my_loc/ndk.rosinstall
 
exit 0