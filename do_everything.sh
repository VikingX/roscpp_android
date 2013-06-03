#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 prefix_path"
    echo "  example: $0 /home/user/my_workspace"
    exit 1
fi

if [ ! -d $1 ]; then
    mkdir -p $1
fi

prefix=$(cd $1 && pwd)

run_cmd() {
    cmd=$1.sh
    shift
    $my_loc/$cmd $@ || die "$cmd $@ died with error code $?"
}

[ -d $prefix/android-ndk-r8e ] || run_cmd get_ndk $prefix
export ANDROID_NDK=$prefix/android-ndk-r8e

[ -d $standalone_toolchain_path ] || run_cmd setup_standalone_toolchain

mkdir -p $prefix/libs

[ -d $prefix/libs/boost_1_47_0 ] || run_cmd get_boost $prefix/libs
[ -d $prefix/libs/bzip2-1.0.6 ] || run_cmd get_bzip2 $prefix/libs
[ -d $prefix/libs/gtest-1.6.0 ] || run_cmd get_gtest $prefix/libs
[ -d $prefix/libs/catkin ] || run_cmd get_catkin $prefix/libs
[ -d $prefix/libs/console_bridge ] || run_cmd get_console_bridge $prefix/libs

run_cmd build_catkin $prefix/libs/catkin $prefix/target
. $prefix/target/setup.bash
run_cmd get_ros_stuff $prefix/libs

run_cmd patch_boost $prefix/libs/boost_1_47_0
run_cmd prepare_boost $prefix/libs/boost_1_47_0
run_cmd build_boost $prefix/libs/boost_1_47_0

run_cmd build_bzip2 $prefix/libs/bzip2-1.0.6
run_cmd build_gtest $prefix/libs/gtest-1.6.0
run_cmd build_console_bridge $prefix/libs/console_bridge
run_cmd build_rosbag
