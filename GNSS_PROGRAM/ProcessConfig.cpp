#include "ProcessCondig.h"

std::string Trim(const std::string& str) 
{
    size_t first = str.find_first_not_of(" \t");
    size_t last = str.find_last_not_of(" \t");
    if (first == std::string::npos || last == std::string::npos)
        return "";
    return str.substr(first, (last - first + 1));
}

//Read configuration information
void ReadConfig(const std::string& file_path, GNSSConfig& config) 
{
    std::ifstream file(file_path);
    std::string line;
    while (getline(file, line)) 
    {
        line = Trim(line);  // 去除行首行尾空格
        if (line.empty()) continue;  // 忽略空行

        std::string value = line.substr(line.find(":") + 1);
        value = Trim(value);  // 去除value的前后空格

        if (line.find("FROM FILE OR COM:") != std::string::npos) 
        {
            config.com_or_file = std::stoi(value);
        }
        else if (line.find("COM SETUP:") != std::string::npos) 
        {
            std::istringstream iss(value);
            iss >> config.com_port >> config.com_baud_rate;
        }
        else if (line.find("IP ADDRESS B:") != std::string::npos) 
        {
            config.ip_address_B = value.substr(0, value.find(":"));
        }
        else if (line.find("IP ADDRESS R:") != std::string::npos)
        {
            config.ip_address_R = value.substr(0, value.find(":"));
        }
        else if (line.find("PORT_R:") != std::string::npos)
        {
            config.port_R = std::stoi(value);
        }
        else if (line.find("PORT_B:") != std::string::npos)
        {
            config.port_B = std::stoi(value);
        }
        else if (line.find("OBSDATA SOURCE FILE(RFILE):") != std::string::npos) 
        {
            config.obsdata_rfile = value;
        }
        else if (line.find("OBSDATA SOURCE FILE(BFILE):") != std::string::npos) 
        {
            config.obsdata_bfile = value;
        }
        else if (line.find("POSITION RESULT FILE:") != std::string::npos) 
        {
            config.position_result_file = value;
        }
        else if (line.find("POSITION DIFF FILE:") != std::string::npos) 
        {
            config.position_diff_file = value;
        }
        else if (line.find("CODE PSEUDORANGE NOISE:") != std::string::npos) 
        {
            config.code_pseudorange_noise = std::stod(value);
        }
        else if (line.find("CODE CARRIER PHASE NOISE:") != std::string::npos)
        {
            config.code_carrier_phase_noise = std::stod(value);
        }
        else if (line.find("THRESHOLD FOR ELEVATION MASK:") != std::string::npos) 
        {
            config.elevation_mask_threshold = std::stod(value);
        }
        else if (line.find("MODE:") != std::string::npos)
        {
            if (value == "RTK")
            {
                config.Mode = RTKMode;
            }
            else
            {
                config.Mode = SPPMode;
            }
        }
    }
}