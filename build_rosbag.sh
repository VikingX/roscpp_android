#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

cmd_exists catkin_make || die 'catkin_make was not found'

if echo $system | grep _64 >/dev/null; then
    host64='-DANDROID_NDK_HOST_X64=YES'
fi

python=$(which python)

cd $CMAKE_PREFIX_PATH/catkin_ws

catkin_make --cmake-args -DCMAKE_TOOLCHAIN_FILE=$standalone_toolchain_path/android.toolchain.cmake \
    -DANDROID_TOOLCHAIN_NAME=$toolchain -DANDROID_NATIVE_API_LEVEL=$platform $host64 \
    -DPYTHON_EXECUTABLE=$python -DBUILD_SHARED_LIBS=0 -DCMAKE_INSTALL_PREFIX=$CMAKE_PREFIX_PATH

cd build && make install
