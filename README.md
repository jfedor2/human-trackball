# Human Trackball

This is Arduino code for making a Bluetooth trackball from a gym ball (that you sit on).

[Demo video](https://www.youtube.com/watch?v=FprARCVnGL4), some more context [on my blog](https://blog.jfedor.org/2020/09/human-trackball.html).

It's meant to run on a board with an nRF52832 chip and an MPU-9250 sensor, like [this one](https://www.aliexpress.com/item/33014422230.html).

It uses [Sparkfun's library](https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library) for getting the sensor data from the MPU-9250, [Adafruit's library](https://github.com/adafruit/Adafruit_AHRS) to perform sensor fusion and Sandeep Mistry's [BLEPeripheral](https://github.com/sandeepmistry/arduino-BLEPeripheral) library for the Bluetooth stuff. Everything runs on Sandeep Mistry's [Arduino core](https://github.com/sandeepmistry/arduino-nRF5) for Nordic's family of chips. I programmed the board using Nordic's [nRF52 DK](https://www.nordicsemi.com/Software-and-Tools/Development-Kits/nRF52-DK) development kit.

Two additional sketches are included:

`MagnetometerCalibration` can be used to calibrate the magnetometer (duh): it sends the calibration data over a simulated serial connection (using Nordic's UART profile, compatible with [Adafruit's app](https://play.google.com/store/apps/details?id=com.adafruit.bluefruit.le.connect)). `BLESerial.cpp` and `BLESerial.h` come from the BLEPeripheral library examples.

`PedalMouseButton` can be used with a Digispark to perform mouse button clicks with a foot switch.

Sadly, there's an issue with the BLEPeripheral library that means that on some (most) operating systems, it only works the first time the device is connected. To make it work again, you have to clear the bonding information on both sides. On the trackball side it's done by connecting pin 28 to ground while turning it on. Pin 25 is conveniently driven low so you can just connect those two (they're the outer two GPIO pins on the previously mentioned board). Or make your own arrangements if you're using a different board.
