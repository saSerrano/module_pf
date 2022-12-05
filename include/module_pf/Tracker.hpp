/**
 * \class Tracker
 * 
 * \brief This class is for modeling the tracking process, is specified by the threshold values that determine the segmentation of objects (st and sa), and their linking across images from different instants of time (dt).
 * 
 * \author $Author: Sergio A. Serrano$
 * 
 * \date $Date: 25/03/19$
 * 
 * Contact: sserrano@inaoep.mx
 */
#include <iostream>
#include <string>
#include <vector>
#include <stdlib.h>
#include <cmath>
#include <algorithm>

//OpenCV
#include <opencv2/opencv.hpp>

//Tracking
#include <module_pf/TrackedObject.hpp>

#define PI 3.14159265

using namespace std;
using namespace cv;

#ifndef TRACKER
#define TRACKER

class Tracker
{
	private:

		//Segmentation and tracking related variables
		float Segmentation_Threshold;
		float Distance_Threshold_;
		int significant_area;

		//Variable used to generate unique ID numbers for the tracked objects
		int idRecord;

	public:

		/**
		 * \brief Default constructor.
		*/
		Tracker();

		/**
		 * \brief Overloaded constructor that initializes the tracker with a set of threshold values.
		 * \param st Threshold value for the segmentation process, in which if two pixels that are neighbors have intensity values whose absolute difference than st, then they are considered to be part of the same object in the depth image.
		 * \param dt Threshold value for the tracking process, in which if the centroid of two TrackedObjects from different time instants are apart form a distance greater than dt, they cannot be the same object.
		 * \param sa Threshold value for the segmentation process, which corresponds to the minimum amount of pixels an object has to have in ordr to be considered a TrackedObject.
		*/
		Tracker(float st, float dt, int sa);

		/**
		 * \brief Method for segementing the independent objects within a scene.
		 * \param img Depth image from which the segmentation process takes place. A single-channel 32-floating point image is expected, in which the pixel's intensity value is interpreted as the Z-distance in meters.
		 * \return Vector of independent objects, represented as TrackedObjects.
		*/
		vector<TrackedObject> extractObjects(Mat &img);

		/**
		 * \brief Method for performing tracking between TrackedObject from two different instants of time. This method will try to associate every object int currObj to one in prevObj, by re-assigning them an already existing ID. If an object successfully acquires an ID from an object in prevObj, we say they are the same object at different instants of time.
		 * \param currObj Vector of TrackedObjects detected in the current instant of time.
		 * \param prevObj Vector of TrackedObjects detected in the previous instant of time.
		*/
		void linkObjects(vector<TrackedObject> &currObj, vector<TrackedObject> &prevObj);

		/**
		 * \brief Get method for segmentation threshold value.
		 * \return Segmentation threshold value.
		*/
		float sThresh();

		/**
		 * \brief Get method for distance threshold value.
		 * \return Distance threshold value.
		*/
		float dThresh();
};

#endif
