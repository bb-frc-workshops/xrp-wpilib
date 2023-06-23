#pragma once

#include <string>

#define DEFAULT_WATCHDOG_TIMEOUT_MS 1000

namespace xrp {
  class Watchdog {
    public:
      Watchdog(std::string name) : 
          _watchdogTimeout(DEFAULT_WATCHDOG_TIMEOUT_MS),
          _name(name) {}

      void feed();
      bool satisfied();
      void setTimeout(unsigned long timeout);


    private:
      unsigned long _lastWatchdogFeedTime;
      unsigned long _watchdogTimeout;

      bool _lastSatisfiedState;

      std::string _name;
  };
}