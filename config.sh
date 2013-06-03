system=$(uname -s | tr 'DL' 'dl')-$(uname -m)
gcc_version=4.6
toolchain=arm-linux-androideabi-$gcc_version
platform=android-14
standalone_toolchain_path=/tmp/android-toolchain
