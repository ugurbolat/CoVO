#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <nlohmann/json.hpp>

enum LOG_LEVEL {
  trace,
  debug,
  info,
  warn,
  error,
  critical,
  none
};


/**
 *  Wrapper function to use switch/case with strings
 * **/
LOG_LEVEL hasLogLevel(std::string const&);


/** 
 *  Gets the current working dir wrt executable file
 * **/
std::string getCurrentWorkingDir();

/** 
 *  Reads COVO_SETTINGS.json and Raw TUM RGB-D dataset 
 *  
 *    if fails, return 0. if succeeds, returns 1.
 * **/
int read_args(int, char**,
    nlohmann::json*,
    std::vector<std::array<std::string, 2>>*,
    std::vector<std::array<double, 2>>*);
