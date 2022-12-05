#include <stdio.h>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <module_pf/ControlPanHead.hpp>

using namespace std;

void panState(const sensor_msgs::JointState::ConstPtr &feedbackPos)
{
	panGovernor.feedbackPos = feedbackPos->position[1];
}

void goalPosition(const std_msgs::Float64::ConstPtr &goal)
{
	validateGoal(goal->data);
}

void rotatePermission(const std_msgs::Bool::ConstPtr &msg)
{
	panGovernor.permissionToRotate = msg->data;
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "module_pf_panhead");
	ros::NodeHandle n;

	//Get kp, kd and refresh rate values
	float kp,kd,bkp,bkd,maxvel,bmaxvel;
	int refresh_rate;
	string perm_topic,panhead_pos_topic;
	n.param<float>("/module_pf/control_kp",kp,0.5);
	n.param<float>("/module_pf/control_kd",kd,0.2);
	n.param<float>("/module_pf/control_base_kp",bkp,0.5);
	n.param<float>("/module_pf/control_base_kd",bkd,0.2);
	n.param<int>("/module_pf/control_rate",refresh_rate,10);
	n.param<float>("/module_pf/control_maxvel",maxvel,0.75);
	n.param<float>("/module_pf/control_base_maxvel",bmaxvel,0.75);
	n.param<string>("/module_pf/control_rotperm_topic",perm_topic,"/module_pf/panhead/permission");
	n.param<string>("/module_pf/panhead_output_topic",panhead_pos_topic,"/module_pf/panhead/position");

	ros::Publisher commandPublisher = n.advertise<sensor_msgs::JointState>("/joint_commands", 10);
	ros::Subscriber subTorsoState = n.subscribe("/rb1_torso_joint_state", 10, panState);
	ros::Subscriber subPosition = n.subscribe(panhead_pos_topic, 10, goalPosition);
	ros::Publisher basevelPublisher = n.advertise<geometry_msgs::Twist>("/rb1/cmd_vel", 10);
	ros::Subscriber subPermission = n.subscribe(perm_topic, 10, rotatePermission);

	ros::Rate loop_rate(refresh_rate);

	initControl(kp,kd,bkp,bkd,maxvel,bmaxvel);

	while(ros::ok())
	{
		sensor_msgs::JointState pan;

		if(panGovernor.newGoal)
		{
			controlCycle();

			pan.header.stamp = ros::Time::now();
			pan.name.push_back("j1_torso");
			pan.name.push_back("j1_head");
			pan.name.push_back("j2_head");
			pan.velocity.push_back(0.0);
			pan.velocity.push_back(panGovernor.output);
			commandPublisher.publish(pan);

			if(panGovernor.permissionToRotate)
			{
				geometry_msgs::Twist vel_msg;
				vel_msg.angular.x = panGovernor.base_output;
				vel_msg.angular.y = panGovernor.base_output;
				vel_msg.angular.z = panGovernor.base_output;
				basevelPublisher.publish(vel_msg);
			}

			if(checkOutput())
			{
				panGovernor.newGoal--;

				if(panGovernor.permissionToRotate)
				{
					geometry_msgs::Twist vel_msg;
					vel_msg.angular.x = 0.0;
					vel_msg.angular.y = 0.0;
					vel_msg.angular.z = 0.0;
					basevelPublisher.publish(vel_msg);
				}
			}
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
