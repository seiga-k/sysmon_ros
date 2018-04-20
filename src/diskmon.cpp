#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int32.h>
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
	do_loop(false),
	proc_name("/proc/mounts")
	{
		nh.getParam("hz", hz);		
		
		std::string str;
		int32_t line(0);
		while (Util::readSingleLine(proc_name, str, line++)) {
			std::string fspath;
			std::string devname;
			if (qi::parse(
				str.cbegin(),
				str.cend(),
				(
				qi::as_string[qi::lit("/dev/") >> +qi::alnum][bp::ref(devname) = qi::_1] >> qi::blank
				>> qi::as_string[+(qi::string("/") >> *(qi::char_ - (qi::lit('/') | qi::blank)))][bp::ref(fspath) = qi::_1] >> qi::blank >> *qi::char_ 
				)
				)) {
				std::cout << devname << " Path : " << fspath << std::endl;
				
				pub_set pubs;
				pubs.pub_avalable = nh.advertise<std_msgs::Int32>(devname + "/avalable", 1);
				pubs.pub_capacity = nh.advertise<std_msgs::Int32>(devname + "/capacity", 1);
				pubs.pub_freerate = nh.advertise<std_msgs::Float32>(devname + "/freerate", 1);
				pub_usages.insert(std::make_pair(fspath, pubs));
			}
			else if (qi::parse(
				str.cbegin(),
				str.cend(),
				(
				qi::as_string[+(qi::char_ - qi::blank)][bp::ref(devname) = qi::_1] >> qi::blank
				>> qi::lit("/") >> qi::blank >> *qi::char_ 
				)
				)) {
				// for if the root fs is not a typical device
				std::cout << "Root fs device : " << devname << std::endl;
				
				pub_set pubs;
				pubs.pub_avalable = nh.advertise<std_msgs::Int32>(devname + "/avalable", 1);
				pubs.pub_capacity = nh.advertise<std_msgs::Int32>(devname + "/capacity", 1);
				pubs.pub_freerate = nh.advertise<std_msgs::Float32>(devname + "/freerate", 1);
				pub_usages.insert(std::make_pair(fspath, pubs));
			}
		}
		

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
		fs::space_info si = fs::space("/media/frl/My Book");
			
			rate.sleep();
		}
	}

private:
	struct pub_set{
		ros::Publisher pub_capacity;
		ros::Publisher pub_avalable;
		ros::Publisher pub_freerate;
	};
	
	ros::NodeHandle nh;
	std::map<std::string, pub_set> pub_usages;
	double hz;
	bool do_loop;
	const std::string proc_name;
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