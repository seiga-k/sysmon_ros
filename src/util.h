#ifndef SYSMON_UTIL_H
#define SYSMON_UTIL_H

#include <ros/ros.h>
#include <ros/console.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <numeric>

namespace Sysmon{

class Util{
public:	    
    static bool readSingleLine(const std::string fname, std::string& outstr, const int32_t line=0) {
        std::ifstream ifs;
        std::string str;
        int32_t cnt(line + 1);

        ifs.open(fname.c_str());
        if (ifs.fail()) {
                ROS_INFO("Could not open a file");
                return false;
        }
        while (cnt--) {
                str.clear();
                std::getline(ifs, str);
                if (ifs.fail()) {
                        // End of file
                        //ROS_INFO("End of file");
                        return false;
                }
        }
        ifs.close();

        outstr = str;
        return true;
    }
};

}

#endif /* SYSMON_UTIL_H */

