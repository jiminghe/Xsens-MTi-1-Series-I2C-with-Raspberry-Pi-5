
//  Copyright (c) 2003-2022 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "mtssp_driver_i2c.h"
#include <string.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <iostream>
#include <cassert>

/*!	\class MtsspDriverI2c
	\brief MtsspDriver for the I2C bus
*/

#define TIMEOUT_MS 100

using namespace std;


/*!	\brief Constructs a MtsspDriverI2c
	\param[in] deviceAddress The 7 bit I2C address of the MTi
*/
MtsspDriverI2c::MtsspDriverI2c(uint8_t deviceAddress,  const char* i2c_device_name)
	: m_deviceAddress(deviceAddress), m_i2c_device_name(i2c_device_name)
{
	// initialize();
}


MtsspDriverI2c::~MtsspDriverI2c() {
    if (m_i2c_fd >= 0) {
        close(m_i2c_fd);
    }
}

void MtsspDriverI2c::initialize() {
	m_i2c_fd = open(m_i2c_device_name, O_RDWR);
    if (m_i2c_fd < 0) {
        std::cerr << "Failed to open I2C device" << std::endl;
    }

    // Set the I2C slave address
    if (ioctl(m_i2c_fd, I2C_SLAVE, m_deviceAddress) < 0) {
        std::cerr << "Failed to set I2C slave address" << std::endl;
        close(m_i2c_fd);
    }
	else
	{
		cout << "MtsspDriverI2c initialize I2C device ok" << endl;
	}
}



/*!	\brief Perform a blocking write transfer on the I2C bus
	\param[in] opcode Opcode to use
	\param[in] data Pointer to data to be written
	\param[in] dataLength Number of data bytes to write
*/
void MtsspDriverI2c::write(uint8_t opcode, uint8_t const* data, int dataLength)
{
	uint8_t transferBuffer[8];
	assert(dataLength < sizeof(transferBuffer));
	transferBuffer[0] = opcode;
	memcpy(&transferBuffer[1], data, dataLength);
	if (::write(m_i2c_fd, data, dataLength) != dataLength) {
		std::cerr << "Failed MtsspDriverI2c::writeRaw" << std::endl;
		close(m_i2c_fd);
    }
}


/*!	\brief Perform a blocking read transfer on the I2C bus
	\param[in] opcode Opcode to use
	\param[out] data Pointer to result buffer
	\param[in] dataLength Number of data bytes to read
*/
void MtsspDriverI2c::read(uint8_t opcode, uint8_t* dest, int dataLength)
{
	// uint8_t buffer[dataLength];

    if (::write(m_i2c_fd, &opcode, 1) != 1) {
        std::cerr << "Failed to write opcode " << static_cast<int>(opcode) << std::endl;
        close(m_i2c_fd);
        // return false;
    }

    // Read the data from the I2C device, Use ::read to specify the global namespace function
    int bytesRead = ::read(m_i2c_fd, dest, dataLength);
	if (bytesRead < 0) {
		std::cerr << "Error reading data from opcode" << opcode << std::endl;
		close(m_i2c_fd);
	}
	else
	{
		uint16_t notify_size = dest[0] | (dest[1] << 8);
        uint16_t meas_size = dest[2] | (dest[3] << 8);
	}

}


/*!	\brief Perform a blocking write transfer on the I2C bus
	\param[in] data Pointer to data to be written
	\param[in] dataLength Number of data bytes to write
*/
void MtsspDriverI2c::writeRaw(uint8_t const* data, int dataLength)
{
	if (::write(m_i2c_fd, data, dataLength) != dataLength) {
		std::cerr << "Failed MtsspDriverI2c::writeRaw" << std::endl;
		close(m_i2c_fd);
    }
}








