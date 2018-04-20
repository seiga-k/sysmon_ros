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
#include <boost/algorithm/string.hpp>

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
				boost::algorithm::replace_all(fspath, "\\040", " ");
				ROS_INFO("Found device %s : mount point : %s", devname.c_str() ,fspath.c_str());
				fs::space_info si;
				try{
					fs::path path(fspath);
					si = fs::space(path);
				}
				catch(fs::filesystem_error &ex){
					ROS_INFO("Device can not read : %s", devname.c_str());
					ROS_INFO("%s", ex.what());
					continue;
				}
				
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
				
				ROS_INFO("Found device %s : mount point : /", devname.c_str());
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
			
			for(auto&& pubs : pub_usages){
				fs::space_info si;
				try{
					si = fs::space(pubs.first);
				}
				catch(fs::filesystem_error &ex){
					ROS_INFO("Device can not read : %s", ex.what());
					continue;
				}
				
				float rate = (float)si.available / (float)si.capacity * 100.;
				std_msgs::Int32 msg_available;
				msg_available.data = si.available;
				pubs.second.pub_avalable.publish(msg_available);
				
				std_msgs::Int32 msg_capacity;
				msg_capacity.data = si.capacity;
				pubs.second.pub_capacity.publish(msg_capacity);
				
				std_msgs::Float32 msg_rate;
				msg_rate.data = rate;
				pubs.second.pub_freerate.publish(msg_rate);				
			}
			
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