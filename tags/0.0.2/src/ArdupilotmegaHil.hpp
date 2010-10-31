/*
 * ArdupilotmegaHil.hpp
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * ArdupilotmegaHil.hpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ArdupilotmegaHil.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef aerotbx_ArdupilotmegaHil_HPP
#define aerotbx_ArdupilotmegaHil_HPP

#include "AsyncSerial.hpp"
#include <iostream>

namespace ardupilotmegacomm
{

class ArdupilotmegaHil
{
private:
	BufferedAsyncSerial serial;
	uint8_t receiveState;
	uint8_t payloadCount;
	uint8_t payload[18];
	static const uint8_t gpsImuHeader[];
	static const uint8_t xplanePacketID = 0x04;

public:
	struct HilFromApm
	{
		int16_t rollServo, pitchServo, throttleServo;
		int16_t rudderServo, wpDistance, bearingError;
		int16_t nextWpAlt, energyError;
		uint8_t wpIndex, controlMode;
	};

	struct HilToApm
	{
		int16_t roll, pitch, heading, airspeed;
	};

	union MsgFromApm
	{
		HilFromApm msg;
		uint8_t bytes[];
	} fromApm;

	union MsgToApm
	{
		HilToApm msg;
		uint8_t bytes[];
	} toApm;


	ArdupilotmegaHil(const std::string & device, const int baudRate);

	void send();
	void receive();
	void print();
	void unpack();
};


} // ardupilotmegacomm

#endif

// vim:ts=4:sw=4
