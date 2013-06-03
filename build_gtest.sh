#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 gtest_source_dir"
    echo "  example: $0 /home/user/my_workspace/gtest-1.6.0"
    exit 1
fi

cmd_exists cmake || die 'cmake was not found'

if echo $system | grep _64 >/dev/null; then
    host64='-DANDROID_NDK_HOST_X64=YES'
fi

prefix=$(cd $1 && pwd)

cd $1
mkdir -p build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=$standalone_toolchain_path/android.toolchain.cmake \
    -DANDROID_TOOLCHAIN_NAME=$toolchain -DANDROID_NATIVE_API_LEVEL=$platform $host64
make

[ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'
cd $CMAKE_PREFIX_PATH && mkdir -p include && cd include
ln -sf $prefix/include/gtest ./
cd ../lib
ln -sf $prefix/libs/armeabi-v7a/lib*.a ./
