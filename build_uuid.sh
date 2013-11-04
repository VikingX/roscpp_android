#!/bin/bash
 
my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh
 
if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 boost_source_dir"
    echo "  example: $0 /home/user/my_workspace/poco-1.4.6p2"
    exit 1
fi
 
prefix=$(cd $1 && pwd)
 
cd $1
 
# Create a stand alone version of the android toolchain
echo Building UUID...
mkdir toolchain/
$ANDROID_NDK/build/tools/make-standalone-toolchain.sh --platform=android-8 --install-dir=./toolchain --ndk-dir=$ANDROID_NDK --system=linux-x86_64
./configure   --host=arm-linux-androideabi --prefix=/usr/local
cd lib/uuid
make
 
[ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'
mkdir -p $CMAKE_PREFIX_PATH/lib
cd $CMAKE_PREFIX_PATH/lib
ln -sf $prefix/lib/uuid/lib*.a ./
mkdir -p ../include/uuid && cd ../include/uuid
ln -sf $prefix/lib/uuid/*.h ./