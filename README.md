These scripts will (hopefully) help you build static libraries
for tf2 for android and setup a sample application.

You will need android SDK installed and the 'android' program
location in the $PATH.

INSTALL
-------

Source ROS (for python tools):

    source /opt/ros/hydro/setup.bash

The `do_everything.sh` script will call all the other scripts
sequentially, you just have to give it a prefix path:

    ./do_everything.sh /path/to/workspace

YOU WILL PROBABLY HAVE TO RUN THIS MULTIPLE TIMES DUE TO PTHREAD LINKING.

You can also run each script individually, most of them have
a minimalistic help string to give you an idea of their parameters.

When finished, the script will give you a few lines of what it did.
If everything went fine, you will be able to do the following:

    cd /path/to/workspace/sample_app
    ant debug

This will build the app. If you want to install it, run the following:

    ant debug install

This will install the app onto a virtual android device running in the
emulator.

To follow what the app does, you will need to read the log. The sdk has
a tool called `adb` located in `$SDK/platform-tools/`, you can follow the
log by running:

    $SDK/platform-tools/adb logcat

Good luck!
