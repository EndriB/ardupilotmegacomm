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
void h_attitude(void * arg, uint8_t messageId, uint8_t messageVersion, void * messageData);
void h_location(void * arg, uint8_t messageId, uint8_t messageVersion, void * messageData);
void h_servos(void * arg, uint8_t messageId, uint8_t messageVersion, void * messageData);

class CommTest
{
private:
	BufferedAsyncSerial serial;
	BinComm::MessageHandler handlerTable[10]; // setting max handler size to 10
	Stream stream;
public:
	BinComm comm;
	int16_t roll,pitch;
	int16_t yaw;
	int32_t longitude, latitude;
	int16_t altitude; 
	int16_t groundSpeed, groundCourse;
	uint16_t timeOfWeek;

	CommTest(const std::string & device, long int baud) :
		serial(device,baud),
		stream(&serial),
		comm(handlerTable,&stream), roll(), pitch(), yaw(),
		longitude(), latitude(), altitude(), groundSpeed(),
		groundCourse(), timeOfWeek()
	{
		int i=0;
		int32_t latitude = 0, longitude = 0;
		int16_t altitude = 0, groundSpeed = 0, groundCourse = 0;
		uint16_t timeOfWeek = 0;
		handlerTable[i].messageID = BinComm::MSG_ATTITUDE;
		handlerTable[i].handler = h_attitude;
		handlerTable[i].arg = this;
		i++;

		handlerTable[i].messageID = BinComm::MSG_SERVOS;
		handlerTable[i].handler = h_servos;
		handlerTable[i].arg = this;
		i++;

		handlerTable[i].messageID = BinComm::MSG_LOCATION;
		handlerTable[i].handler = h_location;
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

void h_attitude(void * arg, uint8_t messageId, uint8_t messageVersion, void * messageData)
{
	CommTest * commTest = (CommTest*)arg;
	commTest->comm.unpack_msg_attitude(commTest->roll,commTest->pitch,commTest->yaw);
	// displays data when you receive a message, but you should access the data
	// for your ground station etc. via the public attributes of CommTest
	std::cout << "roll, pitch, yaw:\t" << commTest->roll << "\t" << commTest->pitch 
		<< "\t" << commTest->yaw << std::endl;
}

void h_location(void * arg, uint8_t messageId, uint8_t messageVersion, void * messageData)
{
	CommTest * commTest = (CommTest*)arg;
		commTest->comm.unpack_msg_location(commTest->latitude,commTest->longitude,
		commTest->altitude,commTest->groundSpeed,commTest->groundCourse,commTest->timeOfWeek);
	// displays data when you receive a message, but you should access the data
	// for your ground station etc. via the public attributes of CommTest
	std::cout << "latitude, longitude, altitude:\t" << commTest->latitude << "\t" 
		<< commTest->longitude << "\t" << commTest->altitude << std::endl;
	std::cout << "ground speed, ground course, time of week:\t" << commTest->groundSpeed << "\t" 
		<< commTest->groundCourse << "\t" << commTest->timeOfWeek << std::endl;
}

void h_servos(void * arg, uint8_t messageId, uint8_t messageVersion, void * messageData)
{
	int16_t servos[8];
	CommTest * commTest = (CommTest*)arg;
		commTest->comm.unpack_msg_servos(servos[0],servos[1],servos[2],servos[3],
				servos[4],servos[5],servos[6],servos[7]);
	// displays data when you receive a message, but you should access the data
	// for your ground station etc. via the public attributes of CommTest
	std::cout << "servos:\t";
	for (int i=0;i<8;i++) std::cout << servos[i] << "\t";
	std::cout << std::endl;
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
	while(1)
	{
		test.update();
		usleep(1000);
	}
	return 0;
}

// vim:ts=4:sw=4
