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
 
if [ -z $ANDROID_NDK ] ; then
    die "ANDROID_NDK ENVIRONMENT NOT FOUND!"
fi
 
[ -d $standalone_toolchain_path ] || run_cmd setup_standalone_toolchain
 
mkdir -p $prefix/libs

# Start with catkin since we use it to build almost everything else
[ -d $prefix/target ] || mkdir -p $prefix/target
export CMAKE_PREFIX_PATH=$prefix/target
 
[ -e $prefix/android.toolchain.cmake ] || ( cd $prefix && download 'https://raw.github.com/taka-no-me/android-cmake/master/android.toolchain.cmake' && cat $my_loc/files/android.toolchain.cmake.addendum >> $prefix/android.toolchain.cmake)
export RBA_TOOLCHAIN=$prefix/android.toolchain.cmake
 
# Now get boost with a specialized build
[ -d $prefix/libs/boost ] || run_cmd get_boost $prefix/libs

[ -d $prefix/libs/poco-1.4.6p2 ] || run_cmd get_poco $prefix/libs
[ -d $prefix/libs/bzip2-1.0.6 ] || run_cmd get_bzip2 $prefix/libs
[ -d $prefix/libs/tinyxml ] || run_cmd get_tinyxml $prefix/libs
[ -d $prefix/libs/uuid ] || run_cmd get_uuid $prefix/libs
[ -d $prefix/libs/OpenCV-2.4.6-android-sdk ] || run_cmd get_opencv $prefix/libs
[ -d $prefix/libs/eigen ] || run_cmd get_eigen $prefix/libs
[ -d $prefix/libs/catkin ] || run_cmd get_catkin $prefix/libs
[ -d $prefix/libs/console_bridge ] || run_cmd get_console_bridge $prefix/libs
[ -d $prefix/libs/pcl ] || run_cmd get_pcl $prefix/libs
[ -d $prefix/libs/eigen ] || run_cmd get_eigen $prefix/libs
[ -d $prefix/libs/flann ] || run_cmd get_flann $prefix/libs
 
run_cmd build_catkin $prefix/libs/catkin
. $prefix/target/setup.bash
run_cmd get_ros_stuff $prefix/libs
 
run_cmd build_bzip2 $prefix/libs/bzip2
run_cmd build_tinyxml $prefix/libs/tinyxml

run_cmd copy_boost $prefix/libs/boost
run_cmd build_poco $prefix/libs/poco-1.4.6p2
run_cmd build_uuid $prefix/libs/uuid
run_cmd build_console_bridge $prefix/libs/console_bridge
run_cmd copy_opencv $prefix/libs/OpenCV-2.4.6-android-sdk
run_cmd build_eigen $prefix/libs/eigen
run_cmd build_flann $prefix/libs/flann
run_cmd build_pcl $prefix/libs/pcl
run_cmd build_cpp

run_cmd setup_ndk_project $prefix/roscpp_android_ndk
run_cmd create_android_mk $prefix/target/catkin_ws/src $prefix/roscpp_android_ndk
( cd $prefix && run_cmd sample_app sample_app $prefix/roscpp_android_ndk )
 
echo
echo 'done.'
echo 'summary of what just happened:'
echo '  target/      was used to build static libraries for ros software'
echo '    include/   contains headers'
echo '    lib/       contains static libraries'
echo '  tf2_ndk/     is a NDK sub-project that can be imported into an NDK app'
echo '  sample_app/  is an example of such an app, a native activity that uses tf2'
echo
echo 'you might now cd into sample_app/, run "ant debug install", and if an'
echo 'android emulator is running, the app will be flashed onto it.'
