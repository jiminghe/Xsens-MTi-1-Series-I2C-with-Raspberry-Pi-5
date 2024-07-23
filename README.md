# Xsens MTi-1 Series I2C with Raspberry Pi 5

## Introduction
This library was tested under `Ubuntu 24.04 LTS` with `Raspberry Pi 5` with `MTi-3-DK`. 

The `WiringPi` was only used to configure the DRDY and RESET pin, while the [Linux I2C](https://github.com/torvalds/linux/blob/master/include/uapi/linux/i2c-dev.h) library was used to read/write the I2C data. 

The `xbus` and `mtinterface` are from the embedded example code from the Xsens MT SDK 2022.0, which were slightly adapted to work with Linux I2C library. And the `xstypes` library is from the Xsens MT SDK 2022.0, which is used to parse the data.

## Install WiringPi

Refer to [this article](https://learn.sparkfun.com/tutorials/raspberry-gpio/c-wiringpi-setup)

```
sudo apt-get purge wiringpi
hash -r
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi
git pull origin
./build
```

At this point, the library should work. Run the gpio command shown below to view some information about the wiringPi version and the Pi that it is running on.
```
gpio -v
gpio readall
```

## Setup the 400kHz I2C Fast Mode

Change the I2C Bus Speed to 400kHz fastmode.
```
find /boot -name "config.txt" | grep config.txt
sudo nano /boot/firmware/config.txt
```
Change this line to:
```
dtparam=i2c_arm=on, i2c_arm_baudrate=400000
```

## Hardware Interface
| Raspberry Pi 5 | MTi-3-DK     |
| -------------- | ------------ |
| 3V3 power      | 3V3(P101-4)  |
| Ground         | GND(P101-6)  |
| GPIO 2(SDA)    | SDA(P100-9)  |
| GPIO 3(SCL)    | SCL(P100-10) |
| GPIO 17        | CTS(P102-4)  |
| GPIO 27        | RESET(P102-5) |

refer to [MTi 1-series DK User Manual](https://mtidocs.movella.com/shield-board) and Raspberry-Pi [GPIO and the 40-pin header](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#gpio)

## How to Use
```
//cd to the mti_rp5 folder
make
sudo ./mti_i2c_rp5
```


## Output Configuration Example command

reference article:
- [How to use Device Data View to learn MT Low Level Communications](https://base.movella.com/s/article/article/How-to-use-Device-Data-View-to-learn-MT-Low-Level-Communications)
- [MT Low Level Communication Protocol Documentation](https://mtidocs.movella.com/mt-low-level-communication-protocol-documentation)

Example:
1) For MTi-1/2/3/7/8, SetOutputConfiguration: SampleTimeFine, Acceleration, RateOfTurn, MagneticField, 100Hz
```
0x10, 0x60, 0xFF, 0xFF, 0x40, 0x20, 0x00, 0x64, 0x80, 0x20, 0x00, 0x64, 0xC0, 0x20, 0x00, 0x64
```
2) For MTi-2/3/7/8, SetOutputConfiguration: SampleTimeFine, EulerAngles, Acceleration, RateOfTurn, MagneticField, StatusWord, 100Hz
```
0x10, 0x60, 0xFF, 0xFF, 0x20, 0x30, 0x00, 0x64, 0x40, 0x20, 0x00, 0x64, 0x80, 0x20, 0x00, 0x64, 0xC0, 0x20, 0x00, 0x64, 0xE0, 0x20, 0xFF, 0xFF
```
3) For MTi-2/3/7/8, SetOutputConfiguration: SampleTimeFine, Quaternion, Acceleration, RateOfTurn, MagneticField, StatusWord, 100Hz
```
0x10, 0x60, 0xFF, 0xFF, 0x20, 0x10, 0x00, 0x64, 0x40, 0x20, 0x00, 0x64, 0x80, 0x20, 0x00, 0x64, 0xC0, 0x20, 0x00, 0x64, 0xE0, 0x20, 0xFF, 0xFF
```
4) For MTi-2/3/7/8, SetOutputConfiguration: SampleTimeFine, Quaternion, Acceleration, RateOfTurn, MagneticField, Free Acceleration, StatusWord, 100Hz
```
0x10, 0x60, 0xFF, 0xFF, 0x20, 0x10, 0x00, 0x64, 0x40, 0x20, 0x00, 0x64, 0x80, 0x20, 0x00, 0x64, 0xC0, 0x20, 0x00, 0x64, 0x40, 0x30, 0x00, 0x64, 0xE0, 0x20, 0xFF, 0xFF
```
5) For MTi-7/8, SampleTimeFine, Acceleration, RateOfTurn, MagneticField, EulerAngles, LatLon(FP1632), AltitudeEllipsoid(FP1632), VelocityXYZ(FP1632), 100Hz
```
0x10, 0x60, 0xFF, 0xFF, 0x40, 0x20, 0x00, 0x64, 0x80, 0x20, 0x00, 0x64, 0xC0, 0x20, 0x00, 0x64, 0x20, 0x30, 0x00, 0x64, 0x50, 0x42, 0x00, 0x64, 0x50, 0x22, 0x00, 0x64, 0xD0, 0x12, 0x00, 0x64
```

You could copy the command and replace the code for the [application.cpp](./src/application.cpp) for this line:
```
uint8_t output_config_payload[] = { };
```
