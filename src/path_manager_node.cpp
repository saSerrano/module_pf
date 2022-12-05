#include <iostream>
#include <ros/ros.h>
#include <module_pf/PathManager.hpp>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc,argv,"module_pf_path_manager_node");

	ros::NodeHandle nh_("~");

	PathManager pm(nh_);

	ros::Rate r(20);// 20Hz

	while(ros::ok())
	{
		ros::spinOnce();

		pm.updatePathRobot();

		r.sleep();
	}

	return 0;
}
