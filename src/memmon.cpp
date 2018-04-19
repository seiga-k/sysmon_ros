/*
 * アイディアメモ
 * /proc/meminfo に全部入ってる
 * 実メモリ・仮想メモリ・統合の3種類をpubすればいい
 */
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

namespace qi = boost::spirit::qi;
namespace bp = boost::phoenix;

namespace Sysmon {

class Memmon {
public:
	Memmon() :
	nh("~"),
	hz(1.0),
	do_loop(false),
	proc_name("/proc/meminfo")
	{
		nh.getParam("hz", hz);
		struct stat f_stat;
		if (stat(proc_name.c_str(), &f_stat) == 0) {
			pub_real_usage = nh.advertise<std_msgs::Float32>("rate_real", 1);
			pub_swap_usage = nh.advertise<std_msgs::Float32>("rate_swap", 1);
			pub_total_usage = nh.advertise<std_msgs::Float32>("rate_total", 1);
		} else {
			ROS_ERROR("Memory usage report is failed to open a %s", proc_name.c_str());
			return;
		}
		do_loop = true;
		ROS_INFO("Start memmon node");
	}

	~Memmon()
	{
	}

	void run()
	{
		ros::Rate rate(hz);
		while (ros::ok() && do_loop) {
			ros::spinOnce();
			
			int32_t real_total;
			int32_t real_free;
			int32_t swap_total;
			int32_t swap_free;
			int32_t total;
			int32_t total_free;
			
			std::string str;
			int comp_count(0);
			int32_t line(0);
			while (Util::readSingleLine(proc_name, str, line++)) {
				if (qi::parse(
					str.cbegin(),
					str.cend(),
					(
					qi::lit("MemTotal:") >> +qi::blank >> +qi::int_[bp::ref(real_total) = qi::_1] >> +qi::blank >> qi::lit("kB") 
					)
					)) {
					comp_count |= 1 << 0;
				}
				else if (qi::parse(
					str.cbegin(),
					str.cend(),
					(
					qi::lit("MemAvailable:") >> +qi::blank >> +qi::int_[bp::ref(real_free) = qi::_1] >> +qi::blank >> qi::lit("kB") 
					)
					)) {
					comp_count |= 1 << 1;
				}
				else if (qi::parse(
					str.cbegin(),
					str.cend(),
					(
					qi::lit("SwapTotal:") >> +qi::blank >> +qi::int_[bp::ref(swap_total) = qi::_1] >> +qi::blank >> qi::lit("kB") 
					)
					)) {
					comp_count |= 1 << 2;
				}
				else if (qi::parse(
					str.cbegin(),
					str.cend(),
					(
					qi::lit("SwapFree:") >> +qi::blank >> +qi::int_[bp::ref(swap_free) = qi::_1] >> +qi::blank >> qi::lit("kB") 
					)
					)) {
					comp_count |= 1 << 3;
				}
			}
			if(comp_count != 0x0F){
				ROS_WARN("meminfo : faild to parse");
				return;
			}
			std_msgs::Float32 msg_real;
			msg_real.data = (1. - (float)real_free / (float)real_total) * 100.;
			std_msgs::Float32 msg_swap;
			msg_swap.data = (1. - (float)swap_free / (float)swap_total) * 100.;
			std_msgs::Float32 msg_total;
			msg_total.data = (1. - (float)(real_free + swap_free) / (float)(real_total + swap_total)) * 100.;
			
			pub_real_usage.publish(msg_real);
			pub_swap_usage.publish(msg_swap);
			pub_total_usage.publish(msg_total);
			
			rate.sleep();
		}
	}

private:
	ros::NodeHandle nh;
	ros::Publisher pub_real_usage;
	ros::Publisher pub_swap_usage;
	ros::Publisher pub_total_usage;
	double hz;
	bool do_loop;
	const std::string proc_name;
};
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "memmon");
	Sysmon::Memmon memmon;
	memmon.run();
	ros::spin();
	return 0;
}