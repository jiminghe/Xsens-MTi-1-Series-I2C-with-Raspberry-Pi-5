
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

#include "mtssp_interface.h"
#include <xbus/xbus.h>
#include <iostream>
#include <cassert>  // for assert()

using namespace std;


/*!	\class MtsspInterface
	\brief Interface for communicating with an Xsens Motion Tracker via MTSSP (MTi Synchronous Serial Protocol)
*/


/*!	\brief Constructs an MtsspInterface
	\param[in] driver The MtsspDriver for handling the communication with the device
*/
MtsspInterface::MtsspInterface(MtsspDriver* driver)
	: m_driver(driver)
{
}


/*!	\brief Read MTSSP protocol info
	\param[out] version: The version byte
	\param[out] dataReadyConfig: The data ready configuration byte
	\sa configureProtocol
*/
void MtsspInterface::readProtocolInfo(uint8_t& version, uint8_t& dataReadyConfig)
{
	uint8_t buffer[2];
	m_driver->read(XBUS_PROTOCOL_INFO, buffer, 2);
	version = buffer[0];
	dataReadyConfig = buffer[1];
}


/*!	\brief Write MTSSP protocol settings
	\param[in] dataReadyConfig The data ready configuration which must be set

	Bit 7:4	Reserved \n
	Bit 3	Measurement pipe DRDY event enable: 0 = disabled, 1 = enabled \n
	Bit 2	Notification pipe DRDY event enable: 0 = disabled, 1 = enabled \n
	Bit 1	Output type of DRDY pin: = 0 Push/pull, 1 = open drain \n
	Bit 0	Polarity of DRDY signal: 0 = Idle low, 1 = Idle high \n
	\sa readProtocolInfo
*/
void MtsspInterface::configureProtocol(uint8_t dataReadyConfig)
{
	m_driver->write(XBUS_CONFIGURE_PROTOCOL, &dataReadyConfig, sizeof(dataReadyConfig));
}


/*!	\brief Read the pipe status
	\param[out] notificationMessageSize: The number of pending notification bytes
	\param[out] measurementMessageSize: The number of pending measurement bytes
*/
void MtsspInterface::readPipeStatus(uint16_t& notificationMessageSize, uint16_t& measurementMessageSize)
{
	uint8_t status[4];
	m_driver->read(XBUS_PIPE_STATUS, status, sizeof(status));
	notificationMessageSize = status[0] | (status[1] << 8);
	measurementMessageSize = status[2] | (status[3] << 8);
	// cout << "mtssp_interface.cpp, notification size = " << notificationMessageSize << endl;
	// cout << "mtssp_interface.cpp, measurement size = " << measurementMessageSize << endl;
}


/*!	\brief Read from notification or measurement data pipe
	\param[out] buffer Result buffer
	\param[in] size Number of bytes to read
	\param[in] pipe Pipe from which to read, XBUS_NOTIFICATION_PIPE or XBUS_MEASUREMENT_PIPE
*/
void MtsspInterface::readFromPipe(uint8_t* buffer, uint16_t size, uint8_t pipe)
{
	assert(pipe == XBUS_NOTIFICATION_PIPE || pipe == XBUS_MEASUREMENT_PIPE);
	m_driver->read(pipe, buffer, size);
}

/*!	\brief Format a message into the raw mtssp format ready for transmission to a motion tracker.
*/
size_t Xbus_createRawMessage(uint8_t* dest, uint8_t const* message, enum XbusBusFormat format)
{
	int n;
	uint8_t checksum;
	uint16_t length;
	uint8_t* dptr = dest;

	length = Xbus_getPayloadLength(message);

	if (dest == 0)
	{
		switch (format)
		{
		case XBF_I2c:
			return (length < 255) ? length + 4 : length + 6;
		case XBF_Spi:
			return (length < 255) ? length + 7 : length + 9;
		case XBF_Uart:
			return (length < 255) ? length + 5 : length + 7;
		}
	}

	switch (format)
	{
	case XBF_I2c:
		*dptr++ = XBUS_CONTROL_PIPE;
		break;

	case XBF_Spi:
		*dptr++ = XBUS_CONTROL_PIPE;
		// Fill bytes required to allow MT to process data
		*dptr++ = 0;
		*dptr++ = 0;
		*dptr++ = 0;
		break;

	case XBF_Uart:
		*dptr++ = XBUS_PREAMBLE;
		*dptr++ = XBUS_MASTERDEVICE;
		break;
	}

	checksum = 0;
	checksum -= XBUS_MASTERDEVICE;

	*dptr = Xbus_getMessageId(message);
	checksum -= *dptr++;

	if (length < XBUS_EXTENDED_LENGTH)
	{
		*dptr = length;
		checksum -= *dptr++;
	}
	else
	{
		*dptr = XBUS_EXTENDED_LENGTH;
		checksum -= *dptr++;
		*dptr = length >> 8;
		checksum -= *dptr++;
		*dptr = length & 0xFF;
		checksum -= *dptr++;
	}

	for (n = 0; n < length; n++)
	{
		*dptr = Xbus_getConstPointerToPayload(message)[n];
		checksum -= *dptr++;
	}

	*dptr++ = checksum;

	return dptr - dest;
}


/*! \brief Sends an xbus message to the motion tracker
	\param[in] xbusMessage Pointer to xbus message which should be send
*/
void MtsspInterface::sendXbusMessage(uint8_t const* xbusMessage)
{
	uint8_t buffer[128];
	size_t rawLength = Xbus_createRawMessage(buffer, xbusMessage, m_driver->busFormat());
	m_driver->writeRaw(buffer, rawLength);
}


