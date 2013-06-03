#!/bin/sh

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 project_path"
    echo "  example: $0 /home/user/my_workspace/rosbag_ndk"
    exit 1
fi

[ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'

if [ ! -d $1 ]; then
    mkdir -p $1
fi

cd $1
ln -s $CMAKE_PREFIX_PATH/include ./
ln -s $CMAKE_PREFIX_PATH/lib ./

cp $my_loc/files/rba/*.mk ./
