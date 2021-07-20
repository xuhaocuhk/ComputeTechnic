/* This file is part of the source code for the following publication:
 * Assembling Self-Supporting Structures, Deuss et al., SIGGRAPH Asia 2014
 *
 * Copyright (C) 2014 Mario Deuss <mario.deuss@epfl.ch>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef SELF_ASSEMBLY_TIMER_H
#define SELF_ASSEMBLY_TIMER_H

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>

namespace MyUtil{

class Timer{

public:
	
	typedef std::chrono::time_point<std::chrono::system_clock> TimePoint;
	typedef std::chrono::duration<double> Duration;
	
	Timer();
	void start();
	void stop();
	double getTimeDouble();
	std::string to_string() const;
	
private:
	TimePoint startTime;
	TimePoint endTime;
};

std::ostream& operator<<(std::ostream& os, const Timer& t);

}

#endif
