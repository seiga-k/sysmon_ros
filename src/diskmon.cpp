#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <numeric>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/xpressive/xpressive.hpp>
#include <boost/phoenix.hpp>
#include <boost/spirit/include/qi.hpp>

#include "util.h"

namespace fs = boost::filesystem;
namespace qi = boost::spirit::qi;
namespace bp = boost::phoenix;

namespace Sysmon {

class Diskmon {
public:
	Diskmon() :
	nh("~"),
	hz(1.0),
	do_loop(false)
	{
		nh.getParam("hz", hz);

		do_loop = true;
		ROS_INFO("Start diskmon node");
	}

	~Diskmon()
	{
	}

	void run()
	{
		ros::Rate rate(hz);
		while (ros::ok() && do_loop) {
			ros::spinOnce();
			
			rate.sleep();
		}
	}

private:
	ros::NodeHandle nh;
	std::map<std::string, ros::Publisher> pub_usages;
	double hz;
	bool do_loop;
};
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "diskmon");
	Sysmon::Diskmon diskmon;
	diskmon.run();
	ros::spin();
	return 0;
}