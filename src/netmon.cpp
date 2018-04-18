#include <ros/ros.h>
#include <ros/console.h>
#include <sysmon_ros/netif.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <numeric>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <linux/if_link.h>

#include "util.h"

namespace Sysmon {

class Netmon {
public:

	Netmon() :
	nh("~"),
	hz(1.0),
	do_loop(true)
	{
		int err_num;
		if (getifaddrs(&ifaddr) == -1) {
			return;
		}

		nh.getParam("hz", hz);

		ifaddrs *ifa_;
		for (ifa_ = ifaddr; ifa_ != NULL; ifa_ = ifa_->ifa_next) {
			if (ifa_->ifa_addr == NULL) {
				continue;
			}

			int family = ifa_->ifa_addr->sa_family;
			std::string ifname(ifa_->ifa_name);

			if (family == AF_INET) {
				char host[NI_MAXHOST];
				err_num = getnameinfo(ifa_->ifa_addr,
					sizeof (sockaddr_in),
					host,
					NI_MAXHOST,
					NULL,
					0,
					NI_NUMERICHOST);
				if (err_num != 0) {
					ROS_WARN("getnameinfo() failed: %s", gai_strerror(err_num));
					return;
				}
				std::string ip(host);
				//std::cout << ifname << " IP : " << ip << std::endl;
				auto it = ips.find(ifname);
				if (it == ips.end()) {
					std::vector<std::string> ip_;
					ip_.push_back(ip);
					ips.insert(std::make_pair(ifname, ip_));
					pub_netifs.insert(std::make_pair(ifname, nh.advertise<sysmon_ros::netif>(ifname, 1)));
					rtnl_link_stats st_tmp;
					if_stats.insert(std::make_pair(ifname, st_tmp));
				} else {
					ips[ifname].push_back(ip);
				}
			}
		}
		for (ifa_ = ifaddr; ifa_ != NULL; ifa_ = ifa_->ifa_next) {
			if (ifa_->ifa_addr == NULL) {
				continue;
			}

			int family = ifa_->ifa_addr->sa_family;
			std::string ifname(ifa_->ifa_name);

			if (family == AF_PACKET && ifa_->ifa_data != NULL) {
				rtnl_link_stats *stats = (rtnl_link_stats *) ifa_->ifa_data;
				if_stats[ifname] = *stats;
			}
		}
		ROS_INFO("Start netmon node");
	}

	~Netmon()
	{
		freeifaddrs(ifaddr);
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

			if (getifaddrs(&ifaddr) == -1) {
				return;
			}

			ifaddrs *ifa_;
			for (ifa_ = ifaddr; ifa_ != NULL; ifa_ = ifa_->ifa_next) {
				if (ifa_->ifa_addr == NULL) {
					continue;
				}

				int family = ifa_->ifa_addr->sa_family;
				std::string ifname(ifa_->ifa_name);

				if (family == AF_PACKET && ifa_->ifa_data != NULL) {
					rtnl_link_stats *stats = (rtnl_link_stats *) ifa_->ifa_data;
					int32_t tx_bps(0.0);
					int32_t rx_bps(0.0);
					double tx_err_rate(0.0);
					double rx_err_rate(0.0);
					sysmon_ros::netif msg;

					tx_bps = (double) (stats->tx_bytes - if_stats[ifname].tx_bytes) * 8.0 / diff_sec;
					rx_bps = (double) (stats->rx_bytes - if_stats[ifname].rx_bytes) * 8.0 / diff_sec;
					if (stats->tx_packets > if_stats[ifname].tx_packets) {
						tx_err_rate = (double) (stats->tx_errors - if_stats[ifname].tx_errors) / (double) (stats->tx_packets - if_stats[ifname].tx_packets) * 100.0;
					}
					if (stats->rx_packets > if_stats[ifname].rx_packets) {
						rx_err_rate = (double) (stats->rx_errors - if_stats[ifname].rx_errors) / (double) (stats->rx_packets - if_stats[ifname].rx_packets) * 100.0;
					}
					msg.if_name = ifname;
					msg.ip = ips[ifname];
					msg.rx_bps = rx_bps;
					msg.rx_error_rate = rx_err_rate;
					msg.tx_bps = tx_bps;
					msg.tx_error_rate = tx_err_rate;
					pub_netifs[ifname].publish(msg);
					ROS_DEBUG("%1.3lf %s\t\trate : %10d, %10d, %3.2f, %3.2f", diff_sec, ifname.c_str(), tx_bps, rx_bps, tx_err_rate, rx_err_rate);

					if_stats[ifname] = *stats;
				}
			}
			last_time = cur_time;

			rate.sleep();
		}

	}

private:
	ros::NodeHandle nh;
	std::map<std::string, ros::Publisher> pub_netifs;
	std::map<std::string, rtnl_link_stats> if_stats;
	std::map<std::string, std::vector<std::string>> ips;
	ifaddrs *ifaddr;
	double hz;
	bool do_loop;
};
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "netmon");
	Sysmon::Netmon netmon;
	netmon.run();
	while (ros::ok());
	return 0;
}