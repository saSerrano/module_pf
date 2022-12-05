#include <module_pf/TrackedObject.hpp>

TrackedObject::TrackedObject():
centroid(),
boundingBox(),
seedPoint(),
id(-1)
{
}

TrackedObject::TrackedObject(Point3f c_, Rect bb_,Point sp_)
{
	centroid = c_;
	boundingBox = bb_;
	seedPoint = sp_;
	id = -1;
}

void TrackedObject::set(Point3f c_, Rect bb_,Point sp_)
{
	centroid = c_;
	boundingBox = bb_;
	seedPoint = sp_;
	id = -1;
}

void TrackedObject::set(TrackedObject &to_)
{
	centroid = to_.centroid;
	boundingBox = to_.boundingBox;
	seedPoint = to_.seedPoint;
	id = to_.id;
}

float TrackedObject::eucD(TrackedObject &to_)
{
	return sqrt(pow(centroid.x - to_.centroid.x,2) + pow(centroid.z - to_.centroid.z,2));
}

float TrackedObject::fullEucD(TrackedObject &to_)
{
	return sqrt(pow(centroid.x - to_.centroid.x,2) + 
		    pow(centroid.y - to_.centroid.y,2) +
		    pow(centroid.z - to_.centroid.z,2));
}

TrackedObject TrackedObject::scale(int s_factor)
{
	TrackedObject scaled_to;
	if(s_factor < 0) return scaled_to;

	scaled_to.set(*(this));
	scaled_to.boundingBox.x *= s_factor;
	scaled_to.boundingBox.y *= s_factor;
	scaled_to.boundingBox.width *= s_factor;
	scaled_to.boundingBox.height *= s_factor;

	return scaled_to;
}

float TrackedObject::overlap(TrackedObject const &t1, TrackedObject const &t2)
{
	Point tl1 = t1.boundingBox.tl();
	Point tl2 = t2.boundingBox.tl();
	Point br1 = Point(tl1.x + t1.boundingBox.width, tl1.y + t1.boundingBox.height);
	Point br2 = Point(tl2.x + t2.boundingBox.width, tl2.y + t2.boundingBox.height);

	int minX,maxX,minY,maxY;
	minX = ((tl1.x > tl2.x) ? tl1.x : tl2.x);
	minY = ((tl1.y > tl2.y) ? tl1.y : tl2.y);
	maxX = ((br1.x < br2.x) ? br1.x : br2.x);
	maxY = ((br1.y < br2.y) ? br1.y : br2.y);

	int int_width = maxX - minX;
	int int_height = maxY - minY;

	if(int_width <= 0 || int_height <= 0) return 0.0;

	int int_area = int_width * int_height;
	int t1_area = t1.boundingBox.width * t1.boundingBox.height;
	int t2_area = t2.boundingBox.width * t2.boundingBox.height;

	float overlap_pct1 = static_cast<float>(int_area) / static_cast<float>(t1_area);
	float overlap_pct2 = static_cast<float>(int_area) / static_cast<float>(t2_area);
	float avg_overlap_pct = (overlap_pct1 + overlap_pct2) / 2;

	return avg_overlap_pct;
}

float TrackedObject::overlap(cv::Rect const &t1, cv::Rect const &t2)
{
	Point tl1 = t1.tl();
	Point tl2 = t2.tl();
	Point br1 = Point(tl1.x + t1.width, tl1.y + t1.height);
	Point br2 = Point(tl2.x + t2.width, tl2.y + t2.height);

	int minX,maxX,minY,maxY;
	minX = ((tl1.x > tl2.x) ? tl1.x : tl2.x);
	minY = ((tl1.y > tl2.y) ? tl1.y : tl2.y);
	maxX = ((br1.x < br2.x) ? br1.x : br2.x);
	maxY = ((br1.y < br2.y) ? br1.y : br2.y);

	int int_width = maxX - minX;
	int int_height = maxY - minY;

	if(int_width <= 0 || int_height <= 0) return 0.0;

	int int_area = int_width * int_height;
	int t1_area = t1.width * t1.height;
	int t2_area = t2.width * t2.height;

	float overlap_pct1 = static_cast<float>(int_area) / static_cast<float>(t1_area);
	float overlap_pct2 = static_cast<float>(int_area) / static_cast<float>(t2_area);
	float avg_overlap_pct = (overlap_pct1 + overlap_pct2) / 2;

	return avg_overlap_pct;
}

bool TrackedObject::closer(TrackedObject const &t1, TrackedObject const &t2)
{
	double d1 = (t1.centroid.x * t1.centroid.x) + (t1.centroid.y * t1.centroid.y) + (t1.centroid.z * t1.centroid.z); 
	double d2 = (t2.centroid.x * t2.centroid.x) + (t2.centroid.y * t2.centroid.y) + (t2.centroid.z * t2.centroid.z); 
	return (d1 < d2);
}
