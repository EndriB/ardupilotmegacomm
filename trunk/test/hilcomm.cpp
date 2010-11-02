/*
 * hilcomm.cpp
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * hilcomm.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * hilcomm.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AsyncSerial.hpp"
#include "ArdupilotmegaHil.hpp"
#include <iostream>
#include <stdexcept>

int main (int argc, char const* argv[])
{
	using namespace ardupilotmegacomm;

	if (argc != 3)
	{
		std::cout << "usage: " << argv[0] << " device baud" << std::endl;
		return 0;
	}
	std::string device = argv[1];
	long int baud = atol(argv[2]);
	std::cout << "device: " << device << std::endl;
	std::cout << "baud: " << baud << std::endl;
	ArdupilotmegaHil * comm;
	try
	{
		comm = new ArdupilotmegaHil(device,baud);
	}
	catch(std::exception & e)
	{
		std::cout << "error: " << e.what() << std::endl;
		return 1;
	}
	double u[4], x[8];

	while(1)
	{
		comm->send();
		comm->receive();
		comm->print();
		usleep(500);
	}
	delete comm;
	return 0;
}

// vim:ts=4:sw=4
