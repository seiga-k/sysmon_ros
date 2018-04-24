#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sysmon_ros/diskinfo.h>

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
	proc_name_mount("/proc/mounts"),
	proc_name_stat("/proc/diskstats")
	{
		nh.getParam("hz", hz);
		
		std::string str;
		int32_t line(0);
		while (Util::readSingleLine(proc_name_mount, str, line++)) {
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
				boost::algorithm::replace_all(fspath, "\\\\", "\\");
				boost::algorithm::replace_all(fspath, "\\010", "\\t");
				boost::algorithm::replace_all(fspath, "\\012", "\\n");
				boost::algorithm::replace_all(fspath, "\\040", " ");
				boost::algorithm::replace_all(fspath, "\\134", "\\");
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

				disk_info di;
				di.name = "/dev/" + devname;
				di.mount_point = fspath;
				di.pub = nh.advertise<sysmon_ros::diskinfo>(devname, 1);
				di.blocksize = 512;
				disk_infos.insert(std::make_pair(devname, di));
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
				disk_info di;
				di.name = devname;
				di.mount_point = "/";
				di.pub = nh.advertise<sysmon_ros::diskinfo>(devname, 1);
				disk_infos.insert(std::make_pair(devname, di));
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
		ros::Time last_time = ros::Time::now();
		ros::Rate rate(hz);
		while (ros::ok() && do_loop) {
			ros::spinOnce();

			ros::Time cur_time = ros::Time::now();
			ros::Duration diff = cur_time - last_time;
			double diff_sec = diff.toSec();
			
			for(auto&& di : disk_infos){
				fs::space_info si;
				int64_t read_blk;
				int64_t wrote_blk;
				try{
					si = fs::space(di.second.mount_point);
				}
				catch(fs::filesystem_error &ex){
					ROS_INFO("Device can not read : %s", ex.what());
					continue;
				}
				
				std::string str;
				int32_t line(0);
				while (Util::readSingleLine(proc_name_stat, str, line++)) {
					std::vector<int64_t> results;
					if (qi::parse(
						str.cbegin(),
						str.cend(),
						(
						+(*qi::blank >> qi::int_) >> +qi::blank >> qi::lit(di.first) >> 
						+(+qi::blank >> qi::int_[bp::push_back(bp::ref(results), qi::_1)])
						)
						)) {
						if(results.size() == 11){
							read_blk = results[2];
							wrote_blk = results[6];
						}

					}
				}
				
				float rate = (float)si.available / (float)si.capacity * 100.;
				sysmon_ros::diskinfo di_msg;
				di_msg.name = di.second.name;
				di_msg.mount_point = di.second.mount_point;
				di_msg.avalable = si.available;
				di_msg.capacity = si.capacity;
				di_msg.free_rate = rate;
				di_msg.read_bps = (float)(read_blk - di.second.read_prev) * 512. / diff_sec;
				di_msg.write_bps = (float)(wrote_blk - di.second.write_prev) * 512. / diff_sec;
				di.second.pub.publish(di_msg);
				
				di.second.read_prev = read_blk;
				di.second.write_prev = wrote_blk;
			}
			last_time = cur_time;
			
			rate.sleep();
		}
	}

private:
	struct disk_info{
		std::string name;
		std::string mount_point;
		int64_t blocksize;
		int64_t write_prev;
		int64_t read_prev;
		ros::Publisher pub;
	};
	
	ros::NodeHandle nh;
	std::map<std::string, disk_info> disk_infos;
	double hz;
	bool do_loop;
	const std::string proc_name_mount;
	const std::string proc_name_stat;
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