#pragma once

class Clock {
  public:
	virtual uint64_t Nanoseconds() = 0;
	virtual double Sleep(double seconds) = 0;
	virtual double SleepUntil(double seconds) = 0;

	double Seconds() {
		return Nanoseconds() * 1e-9;
	}
};
