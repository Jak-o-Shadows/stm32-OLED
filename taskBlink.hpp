/*
 * HeartbeatTask.h
 *
 *  Created on: 03.11.2018
 *      Author: papyk1
 */

#ifndef TASK_HEARTBEATTASK_H_
#define TASK_HEARTBEATTASK_H_

#include "RtosTask.hpp"

class HeartbeatTask: public RtosTask {
private:
	void setup() override;
	void loop() override;
};

#endif