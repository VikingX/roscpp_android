#!/bin/bash
 
my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh
 
if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 prefix_path"
    echo "  example: $0 /home/user/my_workspace"
    exit 1
fi
 
cmd_exists git || die 'git was not found'
 
prefix=$(cd $1 && pwd)
 
official="angles gencpp genlisp genmsg genpy message_generation message_runtime roscpp_core std_msgs common_msgs class_loader"
for p in $official; do
    [ -d $prefix/$p ] || git clone https://github.com/ros/$p.git $prefix/$p
done
 
chadrockey="geometry_experimental ros_comm"
for p in $chadrockey; do
    [ -d $prefix/$p ] || git clone https://github.com/chadrockey/$p.git $prefix/$p
    cd $prefix/$p && git checkout android
done
 
comm="ros"
for p in $comm; do
  [ -d $prefix/$p ] || git clone https://github.com/ros/$p.git $prefix/$p
done
 
 
cd $prefix/ros_comm && git checkout cpp
 
[ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'
 
cd $CMAKE_PREFIX_PATH
mkdir -p catkin_ws/src && cd catkin_ws/src
for p in $official; do
    ln -sf $prefix/$p ./
done

# Which ros components do we need
ln -sf $prefix/ros/tools/rosunit ./

# Make links to which ros_comm components we want on Android
ln -sf $prefix/ros_comm/messages/rosgraph_msgs ./
ln -sf $prefix/ros_comm/messages/std_srvs ./
ln -sf $prefix/ros_comm/tools/rosconsole ./
ln -sf $prefix/ros_comm/utilities/xmlrpcpp ./
ln -sf $prefix/ros_comm/clients/roscpp ./
 
# Make links to which tf2 components we want on Android
ln -sf $prefix/geometry_experimental/tf2_msgs ./
ln -sf $prefix/geometry_experimental/tf2 ./
 
exit 0