#pragma once

#include <ros/ros.h>

#include "time_manager/clock.h"

class RosClock : public Clock {
	uint64_t Nanoseconds() override {
		return ros::Time::now().toNSec();
	}

	double Sleep(double seconds) override {
		double sleep_start = Seconds();
		ros::Duration(seconds).sleep();
		double sleep_end = Seconds();
		return sleep_end - sleep_start;
	}

	double SleepUntil(double seconds) override {
		ros::Time::sleepUntil(ros::Time(seconds));
		return Seconds();
	}
};
