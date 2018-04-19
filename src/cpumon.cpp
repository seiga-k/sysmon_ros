#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <numeric>
#include <sys/stat.h>

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
	do_loop(true),
	proc_name("/proc/stat"),
	cpun(0)
	{
		nh.getParam("hz", hz);
		struct stat f_stat;
		if (stat(proc_name.c_str(), &f_stat) == 0) {
			do_loop = true;
			std::string str;
			int32_t line(0);
			while (Util::readSingleLine(proc_name, str, line++)) {
				int32_t total, worker;
				std::string name;
				if (parse(str, total, worker, name)) {
					pub_usages.insert(std::make_pair(name, nh.advertise<std_msgs::Float32>(name, 1)));
					totals_prev.insert(std::make_pair(name, total));
					workers_prev.insert(std::make_pair(name, worker));
				}
			}

			ROS_INFO("CPU Usage report is enaabled.");
		} else {
			do_loop = false;
			ROS_INFO("CPU Usage report is disabled.");
		}
		
		ROS_INFO("Start cpumon node");
	}

	~Cpumon()
	{
	}

	void run()
	{
		ros::Rate rate(hz);
		while (ros::ok() && do_loop) {
			ros::spinOnce();
			
			std::string str;
			int32_t line(0);
			while (Util::readSingleLine(proc_name, str, line++)) {
				int32_t total, worker;
				std::string name;
				if (parse(str, total, worker, name)) {
					std_msgs::Float32 usage;
					usage.data = (float)(worker - workers_prev[name]) / (float)(total - totals_prev[name]) * 100.;
					pub_usages[name].publish(usage);
					workers_prev[name] = worker;
					totals_prev[name] = total;
				}
			}
			
			rate.sleep();
		}
	}

private:	
	bool parse(const std::string line, int32_t &total, int32_t &worker, std::string &cpu_name) {
		namespace qi = boost::spirit::qi;
		namespace bp = boost::phoenix;

		auto first = line.begin();
		auto last = line.end();

		std::vector<int32_t> results;
		std::string name;

		if (qi::parse(
			first, last,
			(
			qi::as_string[qi::string("cpu") >> *qi::alnum][bp::ref(name) = qi::_1]
			>> +(+qi::blank >> qi::int_[bp::push_back(bp::ref(results), qi::_1)])
			)
			)) {
			//std::cout << "Match!! " << name << " " << results.size() << std::endl;
			if (results.size() == 10) {
				worker = results[0] + results[1] + results[2];
				total = std::accumulate(results.begin(), results.end(), 0);
				cpu_name = name;
			}

			return true;
		}
		return false;
	}
	
	ros::NodeHandle nh;
	std::map<std::string, ros::Publisher> pub_usages;
	std::map<std::string, int32_t> totals_prev;
	std::map<std::string, int32_t> workers_prev;
	double hz;
	bool do_loop;
	int32_t cpun;
	const std::string proc_name;
};
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cpumon");
	Sysmon::Cpumon cpumon;
	cpumon.run();
	ros::spin();
	return 0;
}