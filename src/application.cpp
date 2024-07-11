
//  Copyright (c) 2003-2024 Xsens Technologies B.V. or subsidiaries worldwide.
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

#include "application.h"
#include <mtinterface/mtssp_interface.h>
#include <string>
#include <xbus/xbus.h>
#include <iostream>
#include <iomanip>
#include <wiringPi.h>



#include <xstypes/xsmessage.h>
#include <xstypes/xsversion.h>

using namespace std;

/*!	\class Application
	The main application class of 'example_mti1_i2c_spi_receive_measurement_data'
*/


/*!	\brief Constructs an Application
	\param driver The MtsspDriver for communicating with the MTi
*/
Application::Application(MtsspDriver* driver)
	: m_state(STATE_Idle)
	, m_driver(driver)
{
	m_device = new MtsspInterface(m_driver);
	wiringPiSetupGpio(); // Initialize wiringPi using GPIO numbering
    pinMode(DATA_READY_PIN, INPUT); // Set DRDY_PIN as input
}


void Application::setLiveDataCallback(const std::function<void(const XsDataPacket&)>& callback) {
    onLiveDataAvailable = callback;
}


/*!	\brief Returns the value of the DataReady line
*/
bool checkDataReadyLine()
{
    return digitalRead(DATA_READY_PIN) == HIGH;
}


/*!	\brief Defines the main loop of the program which handles user commands
*/
void Application::run()
{
	handleEvent(EVT_Start);

	while (true)
	{
		if (checkDataReadyLine())
		{
			readDataFromDevice();
		}
	}
}


/*!	\brief Implements the state machine which defines the program
*/
void Application::handleEvent(Event event, const uint8_t* data)
{
	switch (m_state)
	{
		case STATE_Idle:
		{
			if (event == EVT_Start)
			{
				cout<< "Resetting the device" << endl;
				resetDevice();
				m_state = STATE_WaitForWakeUp;
			}
		} break;

		case STATE_WaitForWakeUp:
		{
			if (event == EVT_XbusMessage && Xbus_getMessageId(data) == XMID_Wakeup)
			{
				cout << "Got Wake-up MID from device"  << endl;
				Xbus_message(m_xbusTxBuffer, 0xFF, XMID_GotoConfig, 0);
				m_device->sendXbusMessage(m_xbusTxBuffer);
				m_state = STATE_WaitForConfigMode;
			}
		} break;

		case STATE_WaitForConfigMode:
		{
			// cout << "Application::handleEvent, case STATE_WaitForConfigMode" << endl;
			Xbus_message(m_xbusTxBuffer, 0xFF, XMID_ReqDid, 0);
			m_device->sendXbusMessage(m_xbusTxBuffer);
			m_state = STATE_WaitForDeviceId;
		} break;

		case STATE_WaitForDeviceId:
		{
			if (event == EVT_XbusMessage && Xbus_getMessageId(data) == XMID_DeviceId)
			{
				XsMessage rcv_did(data, sizeof(data));
				uint64_t deviceId = rcv_did.getDataLong();
				if (rcv_did.getDataSize() == 8)
				{
					XsDeviceId did1(deviceId);
					XsDeviceId did2(rcv_did.getDataLong(4));
					if (!did1.isLegacyDeviceId() || !did2.isLegacyDeviceId())//a must-have workaround for Mk5s with FTT firmware
					{
						deviceId = rcv_did.getDataLongLong();
						cout << "Got DeviceId:"  << deviceId << endl;
					}
				}

			
				Xbus_message(m_xbusTxBuffer, 0xFF, XMID_ReqProductCode, 0);
				m_device->sendXbusMessage(m_xbusTxBuffer);
				m_state = STATE_WaitForProductCode;

			}
		} break;

		case STATE_WaitForProductCode:
		{
			if (event == EVT_XbusMessage && Xbus_getMessageId(data) == XMID_ProductCode)
			{

				XsMessage rcv_pdc(data, 20); //20 bytes of valid data
				XsString productCode;

				const char* pc = (const char*) rcv_pdc.getDataBuffer();
				std::string result(pc ? pc : "", rcv_pdc.getDataSize());
				std::string::size_type thingy = result.find(" ");
				if (thingy < rcv_pdc.getDataSize())
					result.erase(result.begin() + (unsigned)thingy, result.end());
				productCode = result;

				cout << "Got Product Code:"  << productCode.toStdString().c_str() << endl;

				Xbus_message(m_xbusTxBuffer, 0xFF, XMID_ReqFirmwareRevision, 0);
				m_device->sendXbusMessage(m_xbusTxBuffer);
				m_state = STATE_WaitForFirmwareRevision;

			}
		} break;

		case STATE_WaitForFirmwareRevision:
		{
			if (event == EVT_XbusMessage && Xbus_getMessageId(data) == XMID_FirmwareRevision)
			{
				XsMessage rcv(data, sizeof(data));
				XsVersion firmwareRevision = XsVersion(rcv.getDataByte(0), rcv.getDataByte(1), rcv.getDataByte(2));

				cout << "Got firmware revision:"  << firmwareRevision.toString() << endl;
				//FA FF C0 0C 40 20 00 64 80 20 00 64 C0 20 00 64 29
				//config Acceleration, RateOfTurn, MagneticField at 100Hz
				// uint8_t output_config_payload[] = {0x40, 0x20, 0x00, 0x64, 0x80, 0x20, 0x00, 0x64, 0xC0, 0x20, 0x00, 0x64};
				//FA FF C0 18 10 60 FF FF 20 30 00 64 40 20 00 64 80 20 00 64 C0 20 00 64 E0 20 FF FF FD
				//config SampleTimeFine, EulerAngles, Acceleration, RateOfTurn, MagneticField, StatusWord at 100Hz.
				cout << "Setting output data to SampleTimeFine, EulerAngles, Acceleration, RateOfTurn, MagneticField, StatusWord. at 100Hz: " << endl;
				uint8_t output_config_payload[] = {0x10, 0x60, 0xFF, 0xFF, 0x20, 0x30, 0x00, 0x64, 0x40, 0x20, 0x00, 0x64, 0x80, 0x20, 0x00, 0x64, 0xC0, 0x20, 0x00, 0x64,0xE0, 0x20, 0xFF, 0xFF};
				size_t payload_size = sizeof(output_config_payload);
				Xbus_message(m_xbusTxBuffer, 0xFF, XMID_SetOutputConfiguration, payload_size);
				uint8_t* payload = Xbus_getPointerToPayload(m_xbusTxBuffer);
				//assign the values from output_config_payload to payload.
				memcpy(payload, output_config_payload, payload_size);
				m_device->sendXbusMessage(m_xbusTxBuffer);
				m_state = STATE_WaitForSetOutputConfigurationAck;

			}
		} break;

		case STATE_WaitForSetOutputConfigurationAck:
		{
			if (event == EVT_XbusMessage && Xbus_getMessageId(data) == XMID_SetOutputConfigurationAck)
			{
				cout << "Output configuration written to device" << endl;
				m_state = STATE_Ready;
				handleEvent(EVT_GotoMeasuring);
			}
		} break;

		case STATE_Ready:
		{

			if (event == EVT_GotoConfig)
			{
				Xbus_message(m_xbusTxBuffer, 0xFF, XMID_GotoConfig, 0);
				m_device->sendXbusMessage(m_xbusTxBuffer);

			}

			if (event == EVT_GotoMeasuring)
			{
				Xbus_message(m_xbusTxBuffer, 0xFF, XMID_GotoMeasurement, 0);
				m_device->sendXbusMessage(m_xbusTxBuffer);

			}

			if (event == EVT_RequestDeviceId)
			{
				Xbus_message(m_xbusTxBuffer, 0xFF, XMID_ReqDid, 0);
				m_device->sendXbusMessage(m_xbusTxBuffer);
			}

			if (event == EVT_XbusMessage)
			{
				
				// Construct XsMessage from m_measurementData
				size_t length = data[3] + 5;
				XsMessage mti_message(data, length);

				// cout<<"mti message id: " << mti_message.getMessageId() << endl;
				if(mti_message.isChecksumOk())
				{
    				// cout << "mti message in hex: " << mti_message.toHexString() << endl;
    				m_packet.setMessage(mti_message);
					//update the m_packet in the callback.
					onLiveDataAvailable(m_packet);
				}

			}

		} break;
	}
}


/*!	\brief Resets the MTi
*/
void Application::resetDevice()
{
	//Software reset: FA FF 40 00 C1
	// cout << "Application::resetDevice sending reset message 40 00 C1" <<endl;
	Xbus_message(m_xbusTxBuffer, 0xFF, XMID_Reset, 0);
	m_device->sendXbusMessage(m_xbusTxBuffer);
}


/*!	\brief Read data from the Notification and Control pipes of the device
*/
void Application::readDataFromDevice()
{
	uint16_t notificationMessageSize;
	uint16_t measurementMessageSize;
	m_device->readPipeStatus(notificationMessageSize, measurementMessageSize);

	m_dataBuffer[0] = XBUS_PREAMBLE;
	m_dataBuffer[1] = XBUS_MASTERDEVICE;

	if (notificationMessageSize && notificationMessageSize < sizeof(m_dataBuffer))
	{	
		// cout<< "notificationMessageSize = " << notificationMessageSize << endl;
		m_device->readFromPipe(&m_dataBuffer[2], notificationMessageSize, XBUS_NOTIFICATION_PIPE);
		handleEvent(EVT_XbusMessage, m_dataBuffer);
	}

	if (measurementMessageSize && measurementMessageSize < sizeof(m_dataBuffer))
	{
		// cout<< "measurementMessageSize = " << measurementMessageSize << endl;
		m_device->readFromPipe(&m_dataBuffer[2], measurementMessageSize, XBUS_MEASUREMENT_PIPE);
		handleEvent(EVT_XbusMessage, m_dataBuffer);
	}
}


