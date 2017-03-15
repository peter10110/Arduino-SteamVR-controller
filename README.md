# Arduino-SteamVR-controller
The arduino code for one of my other projects, where i try to emulate Vive controllers with Leap Motion, and an Arduino based custom built controller

# What is this?
This is the code of my custom built motion controller, used for my other project, where I try to emulat Vive controllers. The buttons and the orientation
are from two of these controllers, and the positional data is from a Leap Motion.

# Known issues
- Currently the orientation data is the built-on DMP's (Digital Motion Processor)  output, without any further processing. It's quite good and stable, 
but it uses only the gyroscope, and the accelerometer, not the magnetometer. With intense movements, it can drift quite easily.
- As the communication is on USB only, you need two, quite long USB cables. This is far from comfortable, especially when using it with a VR headset
(what is the intended use).

# Future plans for improvements
- Add Bluetooth, and a battery, to enable wireless operation.

# Build process
I've built two of these controllers, from an Arduino Nano, an MPU-9255 IMU, and a cheap one-handed controller from ebay.
Some photos from the build process can be found here: http://imgur.com/gallery/ihOr1

# Special thanks
Thanks for github user rpicopter, for the DMP enabled motion processing library for the MPU9250. I used this, to get the DMP data from my MPU-9255.
https://github.com/rpicopter/ArduinoMotionSensorExample
