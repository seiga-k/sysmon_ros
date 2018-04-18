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
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <linux/if_link.h>

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
		struct ifaddrs *ifaddr, *ifa;
		int err_num;
		if(getifaddrs(&ifaddr) == -1){
			return;
		}

		if (stat(proc_name.c_str(), &st_tmp) == 0) {
			std::string str;
			int32_t line(0);
			while (Util::readSingleLine(proc_name, str, line++)) {
				std::string ifname;
				if_stat stat;
				if(parse(str, ifname, stat)){
					if_stats.insert(std::make_pair(ifname, stat));
					pub_netifs.insert(std::make_pair(ifname, nh.advertise<sysmon_ros::netif>(ifname, 1) ) );
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

		int n;
		for (n = 0, ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next, n++) {
			if (ifa->ifa_addr == NULL)
				continue;

			int family = ifa->ifa_addr->sa_family;

			/* For an AF_INET* interface address, display the address */

			if (family == AF_INET) {
				char host[NI_MAXHOST];
				err_num = getnameinfo(	ifa->ifa_addr,
										sizeof (struct sockaddr_in),
										host,
										NI_MAXHOST,
										NULL,
										0,
										NI_NUMERICHOST);
				if (err_num != 0) {
					ROS_WARN("getnameinfo() failed: %s", gai_strerror(err_num));
					return;
				}
				std::string ifname(ifa->ifa_name);
				std::string ip(host);
				std::cout << ifname << " IP : " << ip << std::endl;

			} else if (family == AF_PACKET && ifa->ifa_data != NULL) {
				struct rtnl_link_stats *stats;
				stats = (rtnl_link_stats *)ifa->ifa_data;

				printf("\t\ttx_packets = %10u; rx_packets = %10u\n"
					"\t\ttx_bytes   = %10u; rx_bytes   = %10u\n",
					stats->tx_packets, stats->rx_packets,
					stats->tx_bytes, stats->rx_bytes);
			}
		}

		freeifaddrs(ifaddr);
	}
	
	~Netmon()
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
					sysmon_ros::netif msg;
					tx_bps = (double)(stat.tx_total - if_stats[ifname].tx_total) * 8.0 / diff_sec;
					rx_bps = (double)(stat.rx_total - if_stats[ifname].rx_total) * 8.0 / diff_sec;
					if(stat.tx_pack_total > if_stats[ifname].tx_pack_total){
						tx_err_rate = (double)(stat.tx_err - if_stats[ifname].tx_err) / (double)(stat.tx_pack_total - if_stats[ifname].tx_pack_total) * 100.0;
					}
					if(stat.rx_pack_total > if_stats[ifname].rx_pack_total){
						rx_err_rate = (double)(stat.rx_err - if_stats[ifname].rx_err) / (double)(stat.rx_pack_total - if_stats[ifname].rx_pack_total) * 100.0;
					}
					msg.if_name = ifname;
					msg.rx_bps = rx_bps;
					msg.rx_error_rate = rx_err_rate;
					msg.tx_bps = tx_bps;
					msg.tx_error_rate = tx_err_rate;
					pub_netifs[ifname].publish(msg);
					ROS_DEBUG("%1.3lf %s\t\trate : %10d, %10d, %3.2f, %3.2f", diff_sec, ifname.c_str(), tx_bps, rx_bps, tx_err_rate, rx_err_rate);
					
					if_stats[ifname] = stat;
				}
			}
			last_time = cur_time;
			
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
	std::map<std::string, ros::Publisher> pub_netifs;
	std::map<std::string, Netmon::if_stat> if_stats;
	std::map<std::string, std::string> ips;
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