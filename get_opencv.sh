#!/bin/bash
 
my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh
 
if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 prefix_path"
    echo "  example: $0 /home/user/my_workspace"
    exit 1
fi
 
prefix=$(cd $1 && pwd)
URL=http://jaist.dl.sourceforge.net/project/opencvlibrary/opencv-android/2.4.6/OpenCV-2.4.6-android-sdk-r2.zip
 
download_zip $URL $prefix
