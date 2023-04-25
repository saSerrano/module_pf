/**
 * \class TrackerNode
 * 
 * \brief This class is for performing people tracking based on geometric and appearance features.
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
#include <cstdlib>
#include <cmath>
#include <algorithm>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <boost/signals2/mutex.hpp>
#include <sensor_msgs/JointState.h>

//OpenCV
#include <opencv2/opencv.hpp>

//Tracking
#include <module_pf/TrackedObject.hpp>
#include <module_pf/Tracker.hpp>
#include <module_pf/SSDetector.hpp>

#define SCALE_FACTOR 2

using namespace std;
using namespace cv;

#ifndef TRACKER_NODE
#define TRACKER_NODE

//Filter used to synchronize the reception of color and depth image topics
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> mySyncPolicy;

class TrackerNode
{

	public:
		//Tracker states
		const static int TRACKER_STATE_IDLE = 0;
		const static int TRACKER_STATE_FOLLOWING = 1;
		const static int TRACKER_STATE_FINDING_FOLLOWED_PERSON = 2;

		//Commands that the tracker understands
		const static std::string FOLLOW_PERSON;
		const static std::string STOP_FOLLOWING;

	private:

		//ROS communication variables
		image_transport::ImageTransport it_;
		image_transport::SubscriberFilter dsub_;
		image_transport::SubscriberFilter csub_;
		message_filters::Synchronizer<mySyncPolicy> sync_;
		boost::signals2::mutex mutex_;
		ros::Subscriber comm_sub_;
		ros::Publisher comm_pub_;
		ros::Publisher track_pub_;
		ros::Publisher panhead_pub_;

		//Flag for diplaying the segmented and track image
		bool display_image_flag_;

		//Vector of colors for displaying purposes
		vector<Scalar> colorTable_;

		//Object in charge of performing multi-tracking basedd on the depth image
		TrackerPF tracker_;

		//Object in charge of detecting people within the color image
		SSD ssd_;

		//Vector of TrackedObjects detected in the previous instant of time
		vector<TrackedObject> prev_object_;

		//Overlap threshold bounding boxes from the color and depth image must have to be considered of the same object/person.
		float overlap_thresh_;

		//Flag to enable/disable turning the robot's head
		bool move_head_;

		//ID of the followed person
		int followed_person_id_;

		//Tracker's state variables
		int state_;
		int prev_state_;

	public:

		/**
		 * \brief Overloaded constructor that takes as input a NodeHandle to subscribe to and advertise topics, and the topics for the color and depth sensors.
		 * \param nh_ NodeHandle for the initialisation of publishers and subscribers.
		 * \param color_img_topic Topic of the color image sensor.
		 * \param depth_img_topic Topic of the depth image sensor.
		*/
		TrackerNode(ros::NodeHandle nh_,
			    std::string color_img_topic,
			    std::string depth_img_topic);

		/**
		 * \brief Destructor method.
		*/
		~TrackerNode();

		/**
		 * \brief Callback method for receiving color and depth images.
		 * \param d_msg Pointer of the message holding the depth image.
		 * \param c_msg Pointer of the message holding the color image.
		*/
		void syncCB(const sensor_msgs::ImageConstPtr& d_msg,const sensor_msgs::ImageConstPtr& c_msg);

		/**
		 * \brief Callback method for receiving commands from the master node.
		 * \param msg Pointer of the message holding the master's command.
		*/
		void commCB(const std_msgs::String::ConstPtr& msg);

	private:

		/**
		 * \brief Draws information of the segmentation and tracking processes and displays them in an image.
		 * \param c_img Color image.
		 * \param d_img Depth image.
		 * \param currentObjects Vector of TrackedObjects detected in the depth image.
		 * \param currentPeople Vector of bounding boxes that enclose the detected people in the color image.
		*/
		void displayInfo(Mat &c_img,Mat &d_img,std::vector<TrackedObject> const &currentObjects,std::vector<Rect> const &currentPeople);

		/**
		 * \brief Modifies, if necessary, the bounding box r so it can fit in image img.
		 * \param img Reference image.
		 * \param r Rect (bounding box) to be fitted within the dimensions of img.
		 * \return A Rect that fits in img.
		*/
		cv::Rect correctBox(cv::Mat const &img,cv::Rect const &r);

		/**
		 * \brief Publishes the followed person's relative position to the robot's depth sensor, in the form of a 3D vector. This message is used by the path manager node to navigate the robot until it reaches the person of interest.
		 * \param tracked_o TrackedObject containing the information of the followed person's position.
		 * \param header Time stamp at which TrackedObject was detected.
		*/
		void sendTrack(TrackedObject const &tracked_o,std_msgs::Header header);

		/**
		 * \brief Publishes the desired orientation of the robot's head with respect its torso.
		 * \param cur_obj Vector of detected TrackedObjects in the current instant of time.
		 * \param use_ref If set false, it ignores cur_obj and publishes orientation 0.
		*/
		void moveRobotHead(std::vector<TrackedObject> const &cur_obj,bool use_ref);

		/**
		 * \brief Method in charge of keep tracking the followed person and pass its position to the path manager node.
		 * \param o_vec Vector of TrackedObjects detected in the depth image.
		 * \param header Structure that holds the time-stamp for the other input parameters.
		*/
		void following(std::vector<TrackedObject> &o_vec,std::vector<Rect> &p_vec,std_msgs::Header header);

		/**
		 * \brief Method in charge of finding a new person to follow.
		 * \param o_vec Vector of TrackedObjects detected in the depth image.
		 * \param p_vec Vector of bounding boxes that enclose the detected people in the color image.
		 * \param header Structure that holds the time-stamp for the other input parameters.
		*/
		void searching(std::vector<TrackedObject> &o_vec,std::vector<Rect> &p_vec,std_msgs::Header header);
};

#endif
