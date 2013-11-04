#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 tinyxml_source_dir"
    echo "  example: $0 /home/user/my_workspace/tinyxml"
    exit 1
fi

prefix=$(cd $1 && pwd)
 
cd $1
 
# Create a stand alone version of the android toolchain
echo Building TINYXML...
ndk-build
 
[ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'

mkdir -p $CMAKE_PREFIX_PATH/lib
cd $CMAKE_PREFIX_PATH/lib
ln -sf $prefix/obj/local/armeabi/lib*.a ./
mkdir -p ../include && cd ../include
ln -sf $prefix/jni/include/*.h ./