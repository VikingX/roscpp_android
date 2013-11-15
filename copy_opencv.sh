#!/bin/bash
 
my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh
 
if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 boost_source_dir"
    echo "  example: $0 /home/user/my_workspace/OpenCV-2.4.6-android-sdk-r2"
    exit 1
fi
 
prefix=$(cd $1 && pwd)
 
cd $1

[ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'
mkdir -p $CMAKE_PREFIX_PATH/lib
cd $CMAKE_PREFIX_PATH/lib
ln -sf $prefix/sdk/native/3rdparty/libs/armeabi-v7a/lib*.a ./
ln -sf $prefix/sdk/native/libs/armeabi-v7a/lib*.a ./
mkdir -p ../include && cd ../include
ln -sf $prefix/sdk/native/jni/include/* ./
mkdir -p ../share/OpenCV/cmake && cd ../share/OpenCV/cmake
ln -sf $prefix/sdk/native/jni/OpenCVConfig* ./
cd $CMAKE_PREFIX_PATH
ln -sf $prefix/sdk ./