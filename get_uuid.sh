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
URL=http://downloads.sourceforge.net/project/e2fsprogs/e2fsprogs/v1.42.8/e2fsprogs-1.42.8.tar.gz
 
download_gz $URL $prefix