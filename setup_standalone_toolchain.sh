#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

[ "$ANDROID_NDK" = "" ] && die 'environment variable ANDROID_NDK not set'

ndk_path=$ANDROID_NDK

$ndk_path/build/tools/make-standalone-toolchain.sh --install-dir=$standalone_toolchain_path \
    --system=$system --toolchain=$toolchain --platform=$platform

cd $standalone_toolchain_path
download https://raw.github.com/taka-no-me/android-cmake/master/android.toolchain.cmake
