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

/// @file               WProgram.h
/// @brief              Wrapper for arduino Stream class. 

#include "WProgram.h"

Stream::Stream(BufferedAsyncSerial * serial) : _serial(serial)
{
	if (_serial->errorStatus())
	{
		throw std::runtime_error("WProgram: failed to connect to serial port");
		return;
	}
}

uint8_t Stream::read() {
	char data; 	
	_serial->read(&data,1);
	//std::cout << "avail: " << available() << std::endl;
	return data;
}

void Stream::write(char c)
{
	_serial->write(&c,1);
}

int Stream::available() {
	return _serial->available();
}

extern long int millis()
{
	using namespace boost::posix_time;
	ptime start = microsec_clock::universal_time();
	time_duration diff = microsec_clock::universal_time() - start;
	long int elapsed = diff.total_milliseconds();
	return elapsed;
}
