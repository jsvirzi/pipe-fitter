# Steps

From the project directory, perform the following steps. 
If requested to overwrite, confirm with 'y'.
If not requested, indicating you are not overwriting a file that should exist, 
please check current directory.

    pwd                        # should yield main directory for this project
    cd android
    cp -i ../main.c jni/

Compile/build the Android version of this utility:

    ~/Android/Sdk/ndk-bundle/ndk-build

After building the Android version, prior to pushing the binary onto the device,
it *may* be required to perform the following steps, depending if you are logged on as root
on the device.

    adb disable-verity
    adb reboot

If successful, perform the following steps to load utility onto device

    adb remount
    adb push libs/arm64-v8a/PipeFitter /system/bin/
    adb shell

Once in ADB shell

    PipeFitter -uart /dev/ttyHS2 115200 -server 8086

