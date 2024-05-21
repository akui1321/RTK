#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include "data.h"

/**************************************************************************************************************
  ReadConfig

  Purpose: Read configuration information.
**************************************************************************************************************/

void ReadConfig(const std::string& file_path, GNSSConfig& config);