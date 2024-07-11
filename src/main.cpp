#include <mtinterface/mtssp_driver.h>
#include <mtinterface/mtssp_driver_i2c.h>
#include "application.h"
#include <string>
#include <iostream>
#include <iomanip>

#define MTI_I2C_DEVICE_ADDRESS 0x6B  //the default I2C address for MTi-*-DK


void liveDataHandler(const XsDataPacket& packet) {
    // Process the live data packet
    std::cout << "Received live data packet" << std::endl;

    if(packet.containsSampleTimeFine())
    {
        uint32_t sampleTimeFine = packet.sampleTimeFine();
        std::cout << "SampleTimeFine: " << sampleTimeFine << std::endl;
    }

    if(packet.containsCalibratedAcceleration())
    {
        XsVector acc = packet.calibratedAcceleration();
        std::cout << "Acceleration: " << acc[0] << ", " << acc[1] << ", " << acc[2] << std::endl;
    }

    if(packet.containsCalibratedGyroscopeData())
    {
        XsVector gyro = packet.calibratedGyroscopeData();
        std::cout << "RateOfTurn: " << gyro[0] << ", " << gyro[1] << ", " << gyro[2] << std::endl;
    }

    if(packet.containsCalibratedMagneticField())
    {
        XsVector mag = packet.calibratedMagneticField();
        std::cout << "Mag: " << mag[0] << ", " << mag[1] << ", " << mag[2] << std::endl;
    }

    if(packet.containsOrientation())
    {
        XsEuler euler = packet.orientationEuler();
        std::cout<< "Roll, Pitch, Yaw: " << euler.roll() << ", " << euler.pitch() << ", " << euler.yaw() << std::endl;
    }

    if(packet.containsStatus())
    {
        uint32_t status = packet.status();
        std::cout <<"StatusWord: " << status << std::endl;
    }
}


int main()
{

    MtsspDriver* driver;
    std::string i2c_device_name = "/dev/i2c-1";

    driver = new MtsspDriverI2c(MTI_I2C_DEVICE_ADDRESS, i2c_device_name.c_str());

    Application app(driver);
    app.setLiveDataCallback(liveDataHandler);
    app.run();

}