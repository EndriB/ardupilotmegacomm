// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
// Copyright (c) 2010 James Goppert. All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//        notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
// 
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.      IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
// OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGE.

/// @file               commTest.cpp
/// @brief              Library demonstration. 

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
	uint16_t yaw;
	int32_t longitude, latitude;
	int16_t altitude; 
	uint16_t groundSpeed, groundCourse;
	uint32_t timeOfWeek;

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
		handlerTable[i].messageID = BinComm::msg_ATTITUDE;
		handlerTable[i].handler = h_attitude;
		handlerTable[i].arg = this;
		i++;

		handlerTable[i].messageID = BinComm::msg_SERVOS;
		handlerTable[i].handler = h_servos;
		handlerTable[i].arg = this;
		i++;

		handlerTable[i].messageID = BinComm::msg_LOCATION;
		handlerTable[i].handler = h_location;
		handlerTable[i].arg = this;
		i++;

		//signals end of handle table
		handlerTable[i].messageID = BinComm::msg_NULL;
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

