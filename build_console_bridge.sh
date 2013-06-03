#!/bin/sh

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 console_bridge_source_dir"
    echo "  example: $0 /home/user/my_workspace/console_bridge"
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

[ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'

target=$CMAKE_PREFIX_PATH


cd $1
mkdir -p build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=$standalone_toolchain_path/android.toolchain.cmake \
    -DANDROID_TOOLCHAIN_NAME=$toolchain -DANDROID_NATIVE_API_LEVEL=$platform $host64 \
    -DPYTHON_EXECUTABLE=$python -DCMAKE_INSTALL_PREFIX=$target -DBUILD_SHARED_LIBS=0
make install
