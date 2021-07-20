/* This file is part of the source code for the following publication:
 * Assembling Self-Supporting Structures, Deuss et al., SIGGRAPH Asia 2014
 *
 * Copyright (C) 2014 Mario Deuss <mario.deuss@epfl.ch>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "Timer.h"

namespace MyUtil{

	Timer::Timer(){;}
	void Timer::start(){ startTime = std::chrono::system_clock::now(); }
	void Timer::stop(){ endTime = std::chrono::system_clock::now(); }

	double Timer::getTimeDouble(){
		Duration d = endTime-startTime;
		return d.count();
	}

	std::string Timer::to_string() const {
		Duration d = endTime-startTime;
		std::string s = std::to_string(d.count());
		s.append(" s");
		return s;
	}

	std::ostream& operator<<(std::ostream& os, const Timer& t)
	{
		os << t.to_string();
		return os;
	}

}
