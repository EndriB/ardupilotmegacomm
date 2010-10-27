/*
 * ArdupilotmegaHil.cpp
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * ArdupilotmegaHil.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ArdupilotmegaHil.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ArdupilotmegaHil.hpp"

namespace apmcomm
{

ArdupilotmegaHil::ArdupilotmegaHil(const std::string & device, const int baudRate) :
	serial(device,baudRate), headerCount(0), headerFound(false),
	message(), header(), simState(), apmOutput()
{
	if (serial.errorStatus())
	{
		throw std::runtime_error("ArdupilotmegaHil: failed to connect to serial port");
		return;
	}
	header.push_back('A');
	header.push_back('A');
	header.push_back('A');

	headerOut.push_back(0x44);
	headerOut.push_back(0x49);
	headerOut.push_back(0x59);
	headerOut.push_back(0x64);
}
	
void ArdupilotmegaHil::send(HilToApm & msg)
{
	std::vector<char> buffer = serial.read();

	int i = 0;
	std::cout << "buffer size: " << buffer.size() << std::endl;

	// look for header
	if (!headerFound)
	{
		while(i<buffer.size())
		{
			if(buffer[i++] == header[headerCount]) headerCount++;
			else headerCount = 0;

			if (headerCount >= header.size())
			{
				headerCount = 0;
				headerFound = true;
				break;
			}
		}
	}

	// read message
	if (headerFound)
	{
		std::cout << "header found" << std::endl;
		while(i<buffer.size() && i<packetLength) message.push_back(buffer[i++]);
		if (message.size() == packetLength)
		{
			std::cout << "message found" << std::endl;
			// message complete, read into packet
			for (int i=0;i<packetLength;i++) fromApm.bytes[i] = message[i];
			unpack();
			print();
			headerFound = false;
			//std::cout << "message emptying" << std::endl;
			message.clear();
		}
	}
}

void ArdupilotmegaHil::send()
{
	// add header
	std::vector<char> messageOut;
	pack();
	for (int i=0;i<headerOut.size();i++) messageOut.push_back(headerOut[i]);

	// add xplane packet
	messageOut.push_back(sizeof(MsgToApm));
	messageOut.push_back(xplanePacketID);
	for (int i=0;i<sizeof(MsgToApm)) messageOut.push_back(toApm.bytes[i]);

	// compute checksum
	uint8_t ck_a = 0, ck_b = 0;
	for (int i=headerOut.size();i<messageOut.size();i++)
	{
		ck_a += messageOut[i];
		ck_b += ck_a; 
	}
	messageOut.push_back(ck_a);
	messageOut.push_back(ck_b);

	// output to serial
	serial.write(messageOut);
}

void ArdupilotmegaHil::print()
{
	std::cout
		<< "\nroll:\t\t\t" << fromApm.msg.rollServo
		<< "\npitch:\t\t\t" << fromApm.msg.pitchServo
		<< "\nthrottle:\t\t" << fromApm.msg.throttleServo
		<< "\nrudder:\t\t\t" << fromApm.msg.rudderServo
		<< "\nwaypoint distance:\t" << fromApm.msg.wpDistance
		<< "\nbearing errror:\t\t" << fromApm.msg.bearingError
		<< "\nnext waypoint alt:\t" << fromApm.msg.nextWpAlt
		<< "\nenergy error:\t\t" << fromApm.msg.energyError
		<< "\nwaypoint index:\t\t" << fromApm.msg.wpIndex
		<< "\ncontrol mode:\t\t" << fromApm.msg.controlMode
		<< std::endl;
}

void ArdupilotmegaHil::pack()
{
	toApm.msg.roll = roll*180.0/M_PI*100;
	toApm.msg.pitch = pitch*180.0/M_PI*100;
	toApm.msg.heading = heading*180.0/M_PI*100;
	toApm.msg.airspeed = airspeed*3.2808399*100;
}

void ArdupilotmegaHil::unpack()
{
	rollServo = fromApm.msg.rollServo;
	pitchServo = fromApm.msg.pitchServo;
	throttleServo = fromApm.msg.throttleServo;
	rudderServo = fromApm.msg.rudderServo;
	wpDistance = fromApm.msg.bearingError;
	wpDistance = fromApm.msg.nextWpAlt;
	wpDistance = fromApm.msg.energyError;
	wpDistance = fromApm.msg.controlMode;
}

} // apmcomm

// vim:ts=4:sw=4
