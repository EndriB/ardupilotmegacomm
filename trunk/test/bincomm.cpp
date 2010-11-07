/*
 * bincomm.cpp
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * bincomm.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * bincomm.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AsyncSerial.hpp"
#include "APM_BinComm/APM_BinComm.h"
#include <iostream>

// handlers
void handler(void * arg, uint8_t messageId, uint8_t messageVersion, void * messageData);

class CommTest
{
private:
	BufferedAsyncSerial serial;
	BinComm::MessageHandler handlerTable[10]; // setting max handler size to 10
	Stream stream;
public:
	BinComm comm;
	int16_t roll,pitch;
	uint16_t yaw;
	int32_t longitude, latitude, altitude; 
	uint16_t groundSpeed, groundCourse;
	uint32_t timeOfWeek;
	uint8_t flightMode;
	uint16_t timeStamp, batteryVoltage, commandIndex;

	CommTest(const std::string & device, long int baud) :
		serial(device,baud),
		stream(&serial),
		comm(handlerTable,&stream), roll(), pitch(), yaw(),
		longitude(), latitude(), altitude(), groundSpeed(),
		groundCourse(), timeOfWeek(), flightMode(), timeStamp(),
		batteryVoltage(), commandIndex()
	{
		int i=0;
		handlerTable[i].messageID = BinComm::MSG_ANY;
		handlerTable[i].handler = handler;
		handlerTable[i].arg = this;
		i++;

		//signals end of handle table
		handlerTable[i].messageID = BinComm::MSG_NULL;
	}
	void update()
	{
		comm.update();
	}
};

void handler(void * arg, uint8_t messageId, uint8_t messageVersion, void * messageData)
{
	// displays data when you receive a message, but you should access the data
	// for your ground station etc. via the public attributes of CommTest
	CommTest * commTest = (CommTest*)arg;
	switch(messageId)
	{
		case BinComm::MSG_HEARTBEAT:
			std::cout << "MSG_HEARTBEAT" << std::endl;
			commTest->comm.unpack_msg_heartbeat(commTest->flightMode,commTest->timeStamp,
					commTest->batteryVoltage,commTest->commandIndex);
			std::cout << "\tflightMode: " << int(commTest->flightMode) << std::endl;
			std::cout << "\tbatteryVoltage: " << commTest->batteryVoltage << std::endl;
			break;

		case BinComm::MSG_ACKNOWLEDGE:
			std::cout << "MSG_ACKNOWLEDGE" << std::endl;
			break;

		case BinComm::MSG_ATTITUDE:
			std::cout << "MSG_ATTITUDE" << std::endl;
			commTest->comm.unpack_msg_attitude(commTest->roll,commTest->pitch,commTest->yaw);
						std::cout << "\troll, pitch, yaw:\t" << commTest->roll << "\t" << commTest->pitch 
				<< "\t" << commTest->yaw << std::endl;
			break;

		case BinComm::MSG_LOCATION:
			std::cout << "MSG_LOCATION" << std::endl;
			commTest->comm.unpack_msg_location(commTest->latitude,commTest->longitude,
			commTest->altitude,commTest->groundSpeed,commTest->groundCourse,commTest->timeOfWeek);
			std::cout << "\tlatitude, longitude, altitude:\t" << commTest->latitude/1e7 << "\t" 
				<< commTest->longitude/1e7 << "\t" << commTest->altitude/100 << std::endl;
			std::cout << "\tground speed, ground course, time of week:\t" << commTest->groundSpeed << "\t" 
				<< commTest->groundCourse << "\t" << commTest->timeOfWeek << std::endl;
			break;

		case BinComm::MSG_RADIO_OUT:
			std::cout << "MSG_RADIO_OUT" << std::endl;
			uint16_t radio[8];
			commTest->comm.unpack_msg_radio_out(radio);
			std::cout << "\tradio:\t";
			for (int i=0;i<8;i++) std::cout << radio[i] << "\t";
			std::cout << std::endl;
			break;

		default:
			std::cout << "unhandled message" << std::endl;
	}
}

int main (int argc, char const* argv[])
{
	if (argc != 3)
	{
		std::cout << "usage: " << argv[0] << " device baud" << std::endl;
		return 0;
	}
	std::string device = argv[1];
	long int baud = atol(argv[2]);
	std::cout << "device: " << device << std::endl;
	std::cout << "baud: " << baud << std::endl;
	CommTest test(device,baud);
	int i = 0;
	while(1)
	{
		test.update();
		if (i++ > 100)	
		{
			std::cout << "sending flightplan" << std::endl;
			uint8_t action = 1; // add now
			uint16_t length = 10;
			uint8_t commandID = 0x10;
			for (uint16_t j=0;j<length;j++)
			{
				test.comm.send_msg_command_upload(1,j,length,commandID,0,
						j*1111111,j*1111111,j*1111111);
			}
			i=0;
		}
		usleep(10000);
	}
	return 0;
}

// vim:ts=4:sw=4
