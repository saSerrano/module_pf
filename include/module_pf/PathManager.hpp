/**
 * \class PathManager
 * 
 * \brief This class is for building a path that enables the robot to follow a person, the way this is achieved is by creating and deleting POI's (see homer mapping and navigation documentation) dynamically, using the position of the followed person and the current map as input variables.
 * 
 * \author $Author: Sergio A. Serrano$
 * 
 * \date $Date: 25/03/19$
 * 
 * Contact: sserrano@inaoep.mx
 */
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <cmath>
#include <algorithm>
#include <exception>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <boost/signals2/mutex.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

//tf
#include <tf/tf.h>

//tf2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

//Navigation
#include <homer_mapnav_msgs/StartNavigation.h>
#include <homer_mapnav_msgs/StopNavigation.h>
#include <homer_mapnav_msgs/PointOfInterest.h>
#include <homer_mapnav_msgs/ModifyPOI.h>
#include <homer_mapnav_msgs/DeletePointOfInterest.h>

#ifndef PATH_MANAGER
#define PATH_MANAGER

class PathManager
{
	private:
		//POI name that corresponds to the followed person
		const static std::string FOLLOWED_PERSON_NAME;

		//Commands the path manager understands
		const static std::string PAUSE;
		const static std::string RESUME;
		const static std::string Q_MOVING;

		//ROS communication related variables
		boost::signals2::mutex mutex_;
		ros::Publisher nav_start_pub_;
		ros::Publisher nav_stop_pub_;
		ros::Publisher poi_add_pub_;
		ros::Publisher poi_mod_pub_;
		ros::Publisher poi_del_pub_;
		ros::Subscriber track_sub_;
		ros::Subscriber pose_sub_;
		ros::Subscriber comm_sub_;
		ros::Publisher rotperm_pub_;
		ros::Publisher panhead_pub_;

		//To listen and publish transformations
		tf2_ros::Buffer tf2_buffer_;
		tf2_ros::TransformListener tf2_l_;
		tf2_ros::TransformBroadcaster tf2_b_;

		//Minimum distance that must exist between two POIs within a path (meters)
		float poi_separation_thresh_;

		//Range within the robot is considered has reached a POI within the path (meters)
		float poi_reached_thresh_;

		//Radius around the followed person, within the robot is considered to be close to
		//the person, therefore needs no more to move (meters)
		float person_neig_range_;

		//Range within the robot is considered to be in the same position (meters)
		float not_moving_range_;

		//Period that the robot must stay uninterrupted in the same position, in order for it
		//to be considered static (miliseconds)
		unsigned int not_moving_period_;

		//Flag of the first POI being published
		bool first_poi_sent_;

		//Flag for debugging
		bool debug_;

		//Flag for pausing 
		bool pause_;

		//Flag for publishing the POI of the followed person, so it is displayed in homer_gui
		//(this is for the mere purpose of visualize what is going on, this class does not
		// require to publish it operate properly)
		bool publish_person_poi_;

		//Links involved in mapping the person's position to the map
		std::string depthcam_link_;
		std::string person_link_;

		//The robot's most recent pose value whithin the map
		geometry_msgs::Pose pose_data_;

		//The followed person's most recent position within the map
		homer_mapnav_msgs::PointOfInterest current_person_poi_;

		//List of POIs that define the current path to be navigated to reach the followed person
		std::vector<geometry_msgs::Pose> person_path_;

		//Variables used to determine if the robot is moving or not
		geometry_msgs::Pose pose_ref_;
		std_msgs::Header head_ref_;
		bool robot_is_static_;
		bool first_pose_received_;

	public:

		/**
		 * \brief Constructor that initializes with a NodeHandle.
		 * \param nh_ NodeHandle that is used for subscribing to, and advertising topics.
		*/
		PathManager(ros::NodeHandle nh_);

		/**
		 * \brief Destructor method.
		*/
		~PathManager(){};

		/**
		 * \brief Callback method for receiving the followed person's position, relative to the robot's depth camera sensor.
		 * \param msg Pointer of the message holding the person's position.
		*/
		void trackCB(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

		/**
		 * \brief Callback method for receiving the robot's position within the map
		 * \param msg Pointer of the message holding the robot's position.
		*/
		void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);

		/**
		 * \brief Callback method for receiving commands from the master node.
		 * \param msg Pointer of the message holding the master's command.
		*/
		void commCB(const std_msgs::String::ConstPtr& msg);

		/**
		 * \brief Method for updating the list of POIs that constitute the path to be navigated, based on moves made by the robot, that is, this method removes a POI from the navigation path when it is reached, or all of them when it is close to the followed person.
		*/
		void updatePathRobot();

	private:

		/**
		 * \brief Method to convert a quaternion into its equivalent Raw-Pitch-Yaw vector.
		 * \param q Input quaternion.
		 * \return A vector with three elements, that correspond to the Raw-Pitch-Yaw values.
		*/
		tf::Vector3 q2RPY(tf::Quaternion const &q);

		/**
		 * \brief Method to convert a vector of relative position to the robot's depth sensor to a position within the current map.
		 * \param vec Vector of relative position to the robot's depth sensor.
		 * \param robot_pose Most recent position of the robot within the map.
		 * \param poi Variable that will hold the equivalent position of vec whithin the current map.
		 * \return True if the conversion was performed successfully.
		*/
		bool vec2POI(geometry_msgs::Vector3 const &vec,
			     geometry_msgs::Pose const &robot_pose,
			     homer_mapnav_msgs::PointOfInterest &poi);

		//Ejecuta acciones relacionadas a la navegación, con el objetivo de seguir a
		//la persona de interés, basado en la posición del robot y la de la persona.
		/**
		 * \brief Method for updating the list of POIs that constitute the path to be navigated, based on moves made by the followed person, that is, this method adds a POI to the navigation path when the followed person has distanced herself from the path's most recently added POI.
		 * \param fp_poi The person's current position within the map.
		 * \param robot_pose The robot's current position within the map.
		*/
		void updatePathPerson(homer_mapnav_msgs::PointOfInterest &fp_poi,
				      geometry_msgs::Pose &robot_pose);

		/**
		 * \brief Method to print a tf::Vector3 in console.
		 * \param v Vector to be printed.
		*/
		void printV(tf::Vector3 const &v);

		/**
		 * \brief Method to determine if the robot is currently moving.
		 * \param h Header holding the time-stamp at which pose "p" was received.
		 * \param p The robot's most recent position within the map.
		*/
		void isMoving(std_msgs::Header h, geometry_msgs::Pose p);

		/**
		 * \brief Method to center the robot's head with respect its torso.
		*/
		void centerRobotHead();
};

#endif
