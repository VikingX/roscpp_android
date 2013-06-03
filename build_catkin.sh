#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 2 ]; then
    echo "Usage: $0 catkin_source_dir target_prefix"
    echo "  example: $0 /home/user/my_workspace/catkin /home/user/target"
    exit 1
fi

cmd_exists cmake || die 'cmake was not found'

if echo $system | grep _64 >/dev/null; then
    host64='-DANDROID_NDK_HOST_X64=YES'
fi

if [ ! -d $2 ]; then
    mkdir -p $2
fi

python=$(which python)
target=$(cd $2 && pwd)

cd $1
mkdir -p build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=$standalone_toolchain_path/android.toolchain.cmake \
    -DANDROID_TOOLCHAIN_NAME=$toolchain -DANDROID_NATIVE_API_LEVEL=$platform $host64 \
    -DPYTHON_EXECUTABLE=$python -DCMAKE_INSTALL_PREFIX=$target
make install

echo 'done. please run the following:'
echo "  . $target/setup.bash"

