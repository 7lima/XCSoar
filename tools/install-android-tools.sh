#!/bin/sh

# SDK
mkdir -p ~/opt
cd ~/opt
wget --quiet "http://dl.google.com/android/android-sdk_r23-linux.tgz"
tar xfz android-sdk_r23-linux.tgz
mv android-sdk-linux/ android-sdk-linux_x86/
ln -s android-sdk-linux_x86 android-sdk-linux
rm android-sdk_r23-linux.tgz

# NDK
wget --quiet "http://dl.google.com/android/repository/android-ndk-r14-linux-x86_64.zip"
unzip -q android-ndk-r14-linux-x86_64.zip
rm -f android-ndk-r14-linux-x86_64.zip

export ANDROID_SDK="~/opt/android-sdk-linux_x86"
PATH="$PATH:$ANDROID_SDK_HOME/tools"
PATH="$PATH:$ANDROID_SDK_HOME/platform-tools"

# Android NDK
export ANDROID_NDK="~/opt/android-ndk-r14"
PATH="$PATH:$ANDROID_NDK"

apt-get -y install lib32z1 lib32ncurses5 ant

echo y | ~/opt/android-sdk-linux/tools/android update sdk --no-ui --filter "android-22"
echo y | ~/opt/android-sdk-linux/tools/android update sdk --no-ui --filter "platform-tools-preview"
echo y | ~/opt/android-sdk-linux/tools/android update sdk --no-ui --filter "build-tools-23.0.1"


