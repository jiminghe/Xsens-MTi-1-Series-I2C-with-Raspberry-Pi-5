
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


#ifndef APPLICATION_H
#define APPLICATION_H

#include <cstdint> 
#include <xstypes/xsdatapacket.h>
#include <functional>

#define DATA_READY_PIN				17 // GPIO17 for DRDY line
#define RESET_PIN 					27 //GPIO27 for Reset line

class MtsspDriver;
class MtsspInterface;

enum Event
{
	EVT_Start,
	EVT_GotoConfig,
	EVT_GotoMeasuring,
	EVT_Reset,
	EVT_XbusMessage,
	EVT_RequestDeviceId,
};

enum State
{
	STATE_Idle,
	STATE_WaitForWakeUp,
	STATE_WaitForConfigMode,
	STATE_WaitForDeviceId,
	STATE_WaitForProductCode,
	STATE_WaitForFirmwareRevision,
	STATE_WaitForSetOutputConfigurationAck,
	STATE_Ready,
};

class Application
{
	public:
		Application(MtsspDriver* driver);
		void run();
		void handleEvent(Event event, const uint8_t* data = 0);
		void setLiveDataCallback(const std::function<void(const XsDataPacket&)>& callback);

	private:
		void resetDevice();
		void readDataFromDevice();

		State m_state;
		MtsspDriver* m_driver;
		MtsspInterface* m_device;

		uint8_t m_xbusTxBuffer[256];
		uint8_t m_dataBuffer[256];
		XsDataPacket m_packet;

		std::function<void(const XsDataPacket&)> onLiveDataAvailable;

};


#endif
