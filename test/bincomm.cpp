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
	uint8_t severity;
	char status[50];
	uint32_t interval;
	uint16_t mainLoopCycles;
	uint8_t mainLoopCycleTime, gyroSaturationCount, adcConstraintCount, renormSqrtCount,
			renormBlowupCount, gpsFixCount;
	uint16_t imuHealth, gcsMessageCount;

	CommTest(const std::string & device, long int baud) :
		serial(device,baud),
		stream(&serial),
		comm(handlerTable,512,&stream), roll(), pitch(), yaw(),
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
		case BinComm::MSG_PERF_REPORT:
			commTest->comm.unpack_msg_perf_report(commTest->interval,commTest->mainLoopCycles,
					commTest->mainLoopCycleTime,commTest->gyroSaturationCount,
					commTest->adcConstraintCount,commTest->renormSqrtCount,
					commTest->renormBlowupCount,commTest->gpsFixCount,
					commTest->imuHealth,commTest->gcsMessageCount);
			std::cout << "perf report"
				<< "\n\tmain loop cycle time: " << int(commTest->mainLoopCycleTime)
				<< "\n\tadc constraintt count: " << int(commTest->adcConstraintCount)
				<< "\n\trenorm sqrt count: " << int(commTest->renormSqrtCount)
				<< "\n\trenorm blowup count: " << int(commTest->renormBlowupCount)
				<< "\n\tgps fix count: " << int(commTest->gpsFixCount)
				<< "\n\timu health: " << commTest->imuHealth
				<< "\n\tgcs message count: " << commTest->gcsMessageCount
				<< std::endl;
			break;

		case BinComm::MSG_COMMAND_LIST:
			uint16_t itemNumber, listLength;
			uint8_t commandID;
			uint8_t p1;
			int32_t p2,p3,p4;
			commTest->comm.unpack_msg_command_list(itemNumber,
					listLength,commandID,p1,p2,p3,p4);
			std::cout << "command list: length(" << listLength
				<< ")\titem(" << itemNumber
				<< ")\tp1(" << int(p1)
				<< ")\tlat(" << p2/1.0e7
				<< ")\tlon(" << p3/1.0e7
				<< ")\talt(" << p4/1.0e2
				<< ")" << std::endl;
			break;

		case BinComm::MSG_STATUS_TEXT:
			commTest->comm.unpack_msg_status_text(commTest->severity,commTest->status);
			std::cout << "message: status(" << int(commTest->severity)
				<< ")\t" << commTest->status << std::endl;
			break;

		case BinComm::MSG_HEARTBEAT:
			commTest->comm.unpack_msg_heartbeat(commTest->flightMode,commTest->timeStamp,
					commTest->batteryVoltage,commTest->commandIndex);
			std::cout << "heartbeat: flightMode: " << int(commTest->flightMode)
				<< "\tbatteryVoltage: " << commTest->batteryVoltage << std::endl;
			break;

		case BinComm::MSG_ACKNOWLEDGE:
			uint8_t msgID, sum1, sum2;
			commTest->comm.unpack_msg_acknowledge(msgID,sum1,sum2);
			std::cout << "acknowledged: message(" << std::hex << int(msgID) << std::dec
				<< ")\tsum1(" << int(sum1)
				<< ")\tsum2(" << int(sum2)
				<< ")" << std::endl;
			break;

		case BinComm::MSG_ATTITUDE:
			commTest->comm.unpack_msg_attitude(commTest->roll,commTest->pitch,commTest->yaw);
			std::cout << "attitude: roll(" << commTest->roll 
				<< ")\tpitch(" << commTest->pitch 
				<< ")\tyaw(" << commTest->yaw << ")" 
				<< std::endl;
			break;

		case BinComm::MSG_LOCATION:
			commTest->comm.unpack_msg_location(commTest->latitude,commTest->longitude,
				commTest->altitude,commTest->groundSpeed,commTest->groundCourse,commTest->timeOfWeek);
			std::cout << "location: lat(" << commTest->latitude/1.0e7 
				<< "),\tlon(" << commTest->longitude/1.0e7 
				<< "),\talt(" << commTest->altitude/1.0e2
				<< ")\n\t,ground speed(" << commTest->groundSpeed 
				<< ")\t,ground course(" << commTest->groundCourse 
				<< ")\t,time of week(" << commTest->timeOfWeek 
				<< ")" << std::endl;
			break;

		case BinComm::MSG_RADIO_OUT:
			uint16_t radio[8];
			commTest->comm.unpack_msg_radio_out(radio);
			std::cout << "radio:\t";
			for (int i=0;i<8;i++) std::cout << radio[i] << "\t";
			std::cout << std::endl;
			break;

		case BinComm::MSG_SERVO_OUT:
			int16_t servoRaw[8];
			double servo[8];
			commTest->comm.unpack_msg_servo_out(servoRaw);
			for (int i=0;i<8;i++) servo[i] = servoRaw[i]/100.0;
			std::cout << "servo:\t";
			for (int i=0;i<8;i++) std::cout << servo[i] << "%\t";
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

	
	int fastPeriod=100, slowPeriod=10000, clockFast=millis(), clockSlow=millis();

	while(1)
	{
		int time = millis();

		// fast loop
		if (time-clockFast > fastPeriod)	
		{
			test.comm.update();
			clockFast = time;
		}

		// slop loop
		if (time-clockSlow > slowPeriod)
		{
			std::cout << "sending flightplan" << std::endl;
			uint8_t action = 0; // execute immed. 1, insert in list 0
			uint16_t length = 3;
			uint8_t commandID = 0x10;
			for (uint16_t i=0;i<length;i++)
			{
				if (i==0) commandID = 0x16; // takeoff
				else if (i== length-1 ) commandID = 0x15; // land
				else commandID = 0x10; // navigate to waypoint
				std::cout << "\tuploading waypoint: " << i+1 << std::endl;
				test.comm.send_msg_command_upload(action,i+1,length,commandID,0,i,i,i);
				// can only take 128 bytes at a time, delay before sending more than 3
			}
			clockSlow = time;
		}
	}
	return 0;
}

// vim:ts=4:sw=4
