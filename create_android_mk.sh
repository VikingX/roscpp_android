#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 2 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 project_path"
    echo "  example: $0 /home/user/my_catkin_ws/src /home/user/my_output_library_dir"
    exit 1
fi

[ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'

prefix=$(cd $1 && pwd)

# Get list of packages from catkin
package_list=$(/opt/ros/hydro/bin/catkin_topological_order --only-names $prefix | tr '\n' ';')

# Call a CMAKE script to get the equivalent of $catkin_LIBRARIES for all of the above packages
rm -rf $CMAKE_PREFIX_PATH/find_libs
mkdir -p $CMAKE_PREFIX_PATH/find_libs
cp $my_loc/files/FindLibrariesCMakeLists.txt $CMAKE_PREFIX_PATH/find_libs/CMakeLists.txt
cd $CMAKE_PREFIX_PATH/find_libs
cmake ../find_libs -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH;$ANDROID_NDK/platforms/android-14/arch-arm/usr/lib" \
             -DALL_PACKAGES=$package_list

# Read the output file to get the paths of all of the libraries
full_library_list=$(cat $CMAKE_PREFIX_PATH/find_libs/libraries.txt)

# Parse this libraries (separated by ;), skip all libraries that start with the second argument paths (separated by ;)
output=$($my_loc/parse_libs.py $full_library_list $ANDROID_NDK/platforms/android-14/arch-arm/usr/lib)

# Go to the output library directory
if [ ! -d $2 ]; then
    mkdir -p $2
fi
cd $2

# Create and Android.mk from the output
cp $my_loc/files/tfa/Android.mk.in1 ./Android.mk
echo "$output" >> ./Android.mk
cat $my_loc/files/tfa/Android.mk.in2 >> ./Android.mk