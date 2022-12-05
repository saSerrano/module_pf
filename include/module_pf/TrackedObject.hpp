/**
 * \class TrackedObject
 * 
 * \brief This class is for representing segmented objects from a depth image.
 * 
 * \author $Author: Sergio A. Serrano$
 * 
 * \date $Date: 25/03/19$
 * 
 * Contact: sserrano@inaoep.mx
 */
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#ifndef TRACKED_OBJECT
#define TRACKED_OBJECT

class TrackedObject
{
	public:


		//Vector that represents the object's position relative to the depth sensor.
		Point3f centroid;

		//Bounding box that encloses the object whithin the depth image.
		Rect boundingBox;

		//Variable related to the process followed to segment such object.
		Point seedPoint;

		//The object's identifier number
		int id;

		/**
		 * \brief Default constructor.
		*/
		TrackedObject();

		/**
		 * \brief Overloaded constructor that initialiazes with a specific set of values.
		 * \param c_ Centroid value.
		 * \param bb_ Bounding box value.
		 * \param sp_ Seed point value.
		*/
		TrackedObject(Point3f c_, Rect bb_,Point sp_);

		/**
		 * \brief Set method that uses separate parameters.
		 * \param c_ Centroid value.
		 * \param bb_ Bounding box value.
		 * \param sp_ Seed point value.
		*/
		void set(Point3f c_, Rect bb_,Point sp_);

		/**
		 * \brief Set method that uses another TrackedObject instance to copy its values.
		 * \param to_ Source TrckedObject instance.
		*/
		void set(TrackedObject &to_);

		/**
		 * \brief Method for computing the euclidean distance between this instance and the one passed as input parameter, along the X and Z axes.
		 * \param to_ The other TrackedObject against the distance will be computed.
		 * \return Euclidean distance between his instance and to_.
		*/
		float eucD(TrackedObject &to_);

		/**
		 * \brief Method for computing the euclidean distance between this instance and the one passed as input parameter, along all the three axes.
		 * \param to_ The other TrackedObject against the distance will be computed.
		 * \return Euclidean distance between his instance and to_.
		*/
		float fullEucD(TrackedObject &to_);

		/**
		 * \brief Creates an almost identical copy of this instance, which varies only with respect the dimensions of their bounding box parameter. The BB of the returned is scaled to s_factor with respect this instance's BB.
		 * \param s_factor Scaling factor.
		 * \return TrackedObject with a scaled bounding box.
		*/
		TrackedObject scale(int s_factor);

		/**
		 * \brief Computes the proportion of overlapping the bounding boxes of two Tracked objects have, in which 1 corresponds to a couple of identical bounding boxes, while 0 means they do not overlap at all.
		 * \param t1 TrackedObject one.
		 * \param t2 TrackedObject two.
		 * \return Proportion of overlap, which is in the range [0,1].
		*/
		static float overlap(TrackedObject const &t1, TrackedObject const &t2);

		/**
		 * \brief Computes the proportion of overlapping two bounding boxes, in which 1 corresponds to a couple of identical bounding boxes, while 0 means they do not overlap at all.
		 * \param t1 Bounding box one.
		 * \param t2 Bounding box two.
		 * \return Proportion of overlap, which is in the range [0,1].
		*/
		static float overlap(cv::Rect const &t1, cv::Rect const &t2);

		/**
		 * \brief Compares the tracked objects to determine if t1 is closer than t2, based on their centroids.
		 * \param t1 TrackedObject one.
		 * \param t2 TrackedObject two.
		 * \return True if t1 is closer than t2.
		*/
		static bool closer(TrackedObject const &t1, TrackedObject const &t2);
};

#endif
