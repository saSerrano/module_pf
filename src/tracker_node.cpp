#include <iostream>
#include <ros/ros.h>
#include <module_pf/TrackerNode.hpp>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc,argv,"module_pf_tracker_node");

	ros::NodeHandle nh_("~");

	//Topicos de los que se obtendran las imagenes de color y profundidad
	string color_img_topic;
	string depth_img_topic;
	nh_.param<string>("/module_pf/color_input_topic",color_img_topic,"/orbbec_torso_camera/rgb/image_raw");
	nh_.param<string>("/module_pf/depth_input_topic",depth_img_topic,"/orbbec_torso_camera/depth_registered/image_raw");

	TrackerNode tn(nh_,color_img_topic,depth_img_topic);

	ros::spin();

	return 0;
}
