/**
 * \class SSDetector
 * 
 * \brief This class was implemented with the purpose of loading the SSD neural network to detect people within a color image.
 * 
 * \author $Author: Sergio A. Serrano$
 * 
 * \date $Date: 25/03/19$
 * 
 * Contact: sserrano@inaoep.mx
 */
#ifndef __SSDETECTOR__H
#define __SSDETECTOR__H

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/core.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;
using namespace cv::dnn;


class SSD
{
	public:
		/**
		 * \brief Default constructor, uses default values to initialize the SSD network.
		*/
		SSD();

		/**
		 * \brief Overloaded constructor, takes for input the SSD's parameters files.
		 * \param prototxt Prototype file for the SSD network.
		 * \param model File holding the SSD weight parameters.
		*/
		SSD(string prototxt, string model);

		/**
		 * \brief Destructor method.
		*/
		~SSD(){}

		//Method for detecting people within an image. Returns a vector of bounding boxes,
		//each enclosing a detected person.
		/**
		 * \brief Method for detecting people within an image. Returns a vector of bounding boxes, each enclosing a detected person.
		 * \param frame Image from which people shall be detetcted.
		 * \return A vector holding the the bounding boxes for the detected people.
		*/
		vector<Rect> detectP(Mat& frame);

	private:
		//Neural net related variables.
		string prototxt;
		string caffemodel;
		Net net;

		//Confidence thresh. that must bu surpassed by a detection in order to be considered
		//as a positive one.
		float confidenceThreshold;

		//List of classes known by the mobile SSD network
		vector<string> classes;
};	


#endif
