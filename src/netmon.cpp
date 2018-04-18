#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sysmon_ros/netif.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <numeric>
#include <sys/stat.h>

#include <boost/algorithm/string.hpp>
#include <boost/xpressive/xpressive.hpp>
#include <boost/phoenix.hpp>
#include <boost/spirit/include/qi.hpp>

#include "util.h"

namespace qi = boost::spirit::qi;
namespace bp = boost::phoenix;

namespace Sysmon{
class Netmon{
public:
	Netmon() :
	nh("~"),
	if_cnt(0),
	proc_name("/proc/net/dev"),
	hz(1.0),
	do_loop(true)
	{
		struct stat st_tmp;

		if (stat(proc_name.c_str(), &st_tmp) == 0) {
			std::string str;
			int32_t line(0);
			while (Util::readSingleLine(proc_name, str, line++)) {
				std::string ifname;
				if_stat stat;
				if(parse(str, ifname, stat)){
					if_stats.insert(std::make_pair(ifname, stat));
					if_cnt++;
				}
			}
			ROS_INFO("%d interface found.", if_cnt);
			ROS_INFO("Start network monitor");
		} else {
			do_loop = false;
			ROS_WARN("Failed to start Network monitor. No proc file.");
			return;
		}
		
	}
	
	~Netmon()
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
				std::string ifname;
				if_stat stat;
				if(parse(str, ifname, stat)){
					int32_t tx_bps(0.0);
					int32_t rx_bps(0.0);
					double tx_err_rate(0.0);
					double rx_err_rate(0.0);
					tx_bps = hz * (stat.tx_total - if_stats[ifname].tx_total) * 8;
					rx_bps = hz * (stat.rx_total - if_stats[ifname].rx_total) * 8;
					if(stat.tx_pack_total > if_stats[ifname].tx_pack_total){
						tx_err_rate = (double)(stat.tx_err - if_stats[ifname].tx_err) / (double)(stat.tx_pack_total - if_stats[ifname].tx_pack_total) * 100.0;
					}
					if(stat.rx_pack_total > if_stats[ifname].rx_pack_total){
						rx_err_rate = (double)(stat.rx_err - if_stats[ifname].rx_err) / (double)(stat.rx_pack_total - if_stats[ifname].rx_pack_total) * 100.0;
					}
					ROS_DEBUG("%s rate : %d, %d, %3.2f, %3.2f", ifname.c_str(), tx_bps, rx_bps, tx_err_rate, rx_err_rate);
					
					if_stats[ifname] = stat;
				}
			}
			
			rate.sleep();
		}
		
	}
	
private:
	
	struct if_stat{
		int64_t tx_total;
		int64_t rx_total;
		int64_t tx_pack_total;
		int64_t rx_pack_total;
		int64_t tx_err;
		int64_t rx_err;
	};
	
	bool parse(const std::string line, std::string &ifn, if_stat &stat) {
		auto first = line.begin();
		auto last = line.end();

		std::vector<int32_t> result;
		
		if (qi::parse(
			first, last,
			(
			*qi::blank >>  qi::as_string[*qi::alnum - ':'][bp::ref(ifn) = qi::_1] >> ':'
			>> +(+qi::blank >> qi::int_[bp::push_back(bp::ref(result), qi::_1)])
			)
			)) {
			//std::cout << "Match!! " << result.size() << std::endl;
			if (result.size() == 16) {
				stat.rx_total = result[0];
				stat.rx_pack_total = result[1];
				stat.rx_err = result[2];
				stat.tx_total = result[8];
				stat.tx_pack_total = result[9];
				stat.tx_err = result[10];
			}

			return true;
		}
		return false;
	}
		
	ros::NodeHandle nh;
	std::vector<ros::Publisher> pub_tx_rates;
	std::vector<ros::Publisher> pub_rx_rates;
	std::vector<sysmon_ros::netif> netifs;
	std::map<std::string, Netmon::if_stat> if_stats;
	int32_t if_cnt;	
	std::string proc_name;
	double hz;
	bool do_loop;
};
}

int main(int argc, char** argv){
	ros::init(argc, argv, "netmon");
	Sysmon::Netmon netmon;
	netmon.run();
	while (ros::ok());
	return 0;
}