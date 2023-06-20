#pragma once

#include <vector>

enum NetworkMode { AP, STA };

class XRPNetConfig {
  public:
    NetworkMode mode {NetworkMode::AP};
    std::string defaultAPName {"XRP-WPILib"};
    std::string defaultAPPassword {"0123456789"};
    std::vector< std::pair<std::string, std::string> > networkList;
};

class XRPConfiguration {
  public:
    XRPNetConfig networkConfig;

};

XRPConfiguration loadConfiguration(std::string chipIdent);
NetworkMode configureNetwork(XRPConfiguration config);