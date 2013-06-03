#!/bin/sh

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 gtest_source_dir"
    echo "  example: $0 /home/user/my_workspace/gtest-1.6.0"
    exit 1
fi

cmd_exists make || die 'make was not found'

binpfx=arm-linux-androideabi
CC=$standalone_toolchain_path/bin/$binpfx-gcc
AR=$standalone_toolchain_path/bin/$binpfx-ar
RANLIB=$standalone_toolchain_path/bin/$binpfx-ranlib

prefix=$(cd $1 && pwd)

cd $1
make CC=$CC AR=$AR RANLIB=$RANLIB libbz2.a

[ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'
cd $CMAKE_PREFIX_PATH && mkdir -p include
cp $prefix/bzlib.h include/
cp $prefix/libbz2.a lib/
