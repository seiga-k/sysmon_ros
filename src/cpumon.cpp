#include <ros/ros.h>
#include <ros/console.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <numeric>

#include <boost/algorithm/string.hpp>
#include <boost/xpressive/xpressive.hpp>
#include <boost/phoenix.hpp>
#include <boost/spirit/include/qi.hpp>

#include "util.h"

namespace Sysmon {

class Cpumon {
public:

	Cpumon() :
	nh("~"),
	hz(1.0),
	do_loop(true)
	{
		nh.getParam("hz", hz);
		ROS_INFO("Start cpumon node");
	}

	~Cpumon()
	{
	}

	void run()
	{
		ros::Time last_time = ros::Time::now();
		ros::Rate rate(hz);
		while (ros::ok() && do_loop) {
			ros::spinOnce();

			ros::Time cur_time = ros::Time::now();
			ros::Duration diff = cur_time - last_time;
			double diff_sec = diff.toSec();
			
			last_time = cur_time;
			rate.sleep();
		}
	}

private:
	ros::NodeHandle nh;
	double hz;
	bool do_loop;
};
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cpumon");
	Sysmon::Cpumon cpumon;
	cpumon.run();
	while (ros::ok());
	return 0;
}