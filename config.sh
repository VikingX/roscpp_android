system=$(uname -s | tr 'DL' 'dl')-$(uname -m)
gcc_version=4.6
toolchain=arm-linux-androideabi-$gcc_version
platform=android-14
PYTHONPATH=/opt/ros/hydro/lib/python2.7/dist-packages:$PYTHONPATH