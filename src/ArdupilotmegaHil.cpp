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

namespace ardupilotmegacomm
{

const uint8_t ArdupilotmegaHil::gpsImuHeader[] = {'D','I','Y','d'};

ArdupilotmegaHil::ArdupilotmegaHil(const std::string & device, const int baudRate) :
	serial(device,baudRate), receiveState(0), payloadCount(0), fromApm(), toApm()
{
	if (serial.errorStatus())
	{
		throw std::runtime_error("ArdupilotmegaHil: failed to connect to serial port");
		return;
	}
}
	
void ArdupilotmegaHil::receive()
{
	std::vector<char> data = serial.read();

	for(int i=0;i<data.size();i++)
	{

		switch(receiveState)
		{
			case 0:
			case 1:
			case 2:
				if (data[i] == 'A')
					receiveState++;
				else
					receiveState=0;
				break;
			case 3:
				if (payloadCount >= sizeof(payload)) 
				{
					payloadCount = 0;
					receiveState=0;
				}
				else
				{
					payload[payloadCount++] = data[i];
					if (payloadCount == sizeof(payload))
						receiveState++;
				}
				break;
			case 4:
				if (data[i] == '\r')
					receiveState++;
				else
					receiveState=0;
				break;
			case 5:
				if (data[i] == '\n')
				{
					unpack();
					//print();
				}
				receiveState=0;
		}
	}
}

void ArdupilotmegaHil::unpack()
{
	for (int i=0;i<sizeof(MsgFromApm);i++) fromApm.bytes[i] = payload[i];
}

void ArdupilotmegaHil::send()
{
	// add header
	std::vector<char> messageOut;
	for (int i=0;i<sizeof(gpsImuHeader);i++) messageOut.push_back(gpsImuHeader[i]);

	// add xplane packet
	messageOut.push_back(sizeof(MsgToApm));
	messageOut.push_back(xplanePacketID);
	for (int i=0;i<sizeof(MsgToApm);i++) messageOut.push_back(toApm.bytes[i]);

	// compute checksum
	uint8_t ck_a = 0, ck_b = 0;
	for (int i=sizeof(gpsImuHeader);i<messageOut.size();i++)
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
		<< "\nroll servo:\t\t\t" << fromApm.msg.rollServo
		<< "\npitch servo:\t\t\t" << fromApm.msg.pitchServo
		<< "\nthrottle servo:\t\t" << fromApm.msg.throttleServo
		<< "\nrudder servo:\t\t\t" << fromApm.msg.rudderServo
		<< "\nwaypoint distance:\t" << fromApm.msg.wpDistance
		<< "\nbearing errror:\t\t" << fromApm.msg.bearingError
		<< "\nnext waypoint alt:\t" << fromApm.msg.nextWpAlt
		<< "\nenergy error:\t\t" << fromApm.msg.energyError
		<< "\nwaypoint index:\t\t" << fromApm.msg.wpIndex
		<< "\ncontrol mode:\t\t" << fromApm.msg.controlMode
		<< std::endl;
}

} // ardupilotmegacomm

// vim:ts=4:sw=4
