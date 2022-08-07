#pragma once

#include <memory>
#include "time_manager/clock.h"

class TimeManager {
  public:    
    static TimeManager& GetInstance() {
    	static TimeManager instance;
    	return instance;
    }

	void SetClock(std::shared_ptr<Clock>& clock) {
		clock_ = clock;
	}

    Clock* GetClock() {
        return clock_.get();
    }
 
  private:
    TimeManager() = default;
    ~TimeManager() = default;
 
    TimeManager(const TimeManager&) = delete;
    TimeManager& operator=(const TimeManager&) = delete;
    TimeManager(TimeManager&&) = delete;
    TimeManager& operator=(TimeManager&&) = delete;

    std::shared_ptr<Clock> clock_;
};

// // in ros wrapper
// TimeManager::GetInstance().SetClock(new RosClock());

// // in client library
// TimeManager::GetInstance().Sleep(2.0);
