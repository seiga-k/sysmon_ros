/*
 * アイディアメモ
 * /sys/class/hwmon/hwmon* /name にセンサの名前が入ってる
 * /sys/class/hwmon/hwmon* /temp?_input に温度が入ってる
 * /sys/class/hwmon/hwmon* /temp?_label に温度の補助名が入ってる
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

class Tempmon {
public:

	Tempmon() :
	nh("~"),
	hz(1.0),
	do_loop(false),
	hwmon_root("/sys/class/hwmon/"),
	hwmon_num(0)
	{
		try{
			if(!fs::exists(hwmon_root)){
				ROS_ERROR("No hwmon directory");
				return;
			}
			
			for(auto&& hwmons : fs::directory_iterator(hwmon_root)){
				//std::cout << "Path " << hwmons.path() << std::endl;
				
				std::string name;
				fs::path dev_name_path(hwmons.path() / "name");
				if(fs::exists(dev_name_path)){
					Util::readSingleLine(dev_name_path.generic_string(), name);
					//std::cout << "Name : " << name << std::endl;
				}else{
					ROS_ERROR("Unknown style device. tempmon need name of hwmon device.");
					return;
				}
				
				std::string label_fname;
				
				for(auto&& p : fs::directory_iterator(hwmons.path())){
					std::string input_fname(p.path().filename().generic_string());
					int input_num;
					std::string dev_name(name);
					temp_dev dev;
					
					if (qi::parse(
						input_fname.cbegin(),
						input_fname.cend(),
						(
						qi::lit("temp") >> +qi::int_[bp::ref(input_num) = qi::_1] >> qi::lit("_input") 
						)
						)) {
						
						label_fname = "temp" + std::to_string(input_num) + "_label";
						fs::path label_path(hwmons.path() / label_fname);
						if(fs::exists(label_path)){
							std::string label;
							Util::readSingleLine(label_path.generic_string(), label);
							dev_name += "/" + label;
						}
						std::replace(dev_name.begin(), dev_name.end(), ' ', '_');
						//std::cout << "input path : " << p.path() << std::endl;
						dev.temp_file = p.path();
						dev.pub_temp = nh.advertise<std_msgs::Float32>(dev_name, 1);
						temps.push_back(dev);
					}
				}
			}
		}
		catch(const fs::filesystem_error &ex){
			ROS_ERROR("boost::filesystem got error");
		}
		do_loop = true;
		ROS_INFO("Start tempmon node");
	}

	~Tempmon()
	{
	}

	void run()
	{
		ros::Rate rate(hz);
		while (ros::ok() && do_loop) {
			ros::spinOnce();
			
			for(auto&& dev : temps){
				std_msgs::Float32 msg;
				read_temp(dev, msg);
				dev.pub_temp.publish(msg);
			}
			
			rate.sleep();
		}

	}

private:
	struct temp_dev{
		ros::Publisher pub_temp;
		fs::path temp_file;
	};
	
	void read_temp(const temp_dev &p_dev, std_msgs::Float32 &msg){
		std::string temp_str;
		Util::readSingleLine(p_dev.temp_file.generic_string(), temp_str);
		float temp;
		try {
			temp = boost::lexical_cast<double>(temp_str) / 1000.0;
		} catch (const boost::bad_lexical_cast& e) {
			ROS_WARN("lexical_cast error : %s", temp_str.c_str());
			return;
		}
		
		msg.data = temp;
	}
	
	ros::NodeHandle nh;
	std::vector<temp_dev> temps;
	double hz;
	bool do_loop;
	fs::path hwmon_root;
	int hwmon_num;
};
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tempmon");
	Sysmon::Tempmon tempmon;
	tempmon.run();
	ros::spin();
	return 0;
}