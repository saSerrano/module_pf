#include <module_pf/SSDetector.hpp>

SSD::SSD()
{
	this->prototxt = "";
	this->caffemodel = "";
	this->confidenceThreshold = 0.2;

	this->classes.push_back("background");
	this->classes.push_back("aeroplane");
	this->classes.push_back("bicycle");
	this->classes.push_back("bird");
	this->classes.push_back("boat");
	this->classes.push_back("bottle");
	this->classes.push_back("bus");
	this->classes.push_back("car");
	this->classes.push_back("cat");
	this->classes.push_back("chair");
	this->classes.push_back("cow");
	this->classes.push_back("diningtable");
	this->classes.push_back("dog");
	this->classes.push_back("horse");
	this->classes.push_back("motorbike");
	this->classes.push_back("person");
	this->classes.push_back("pottedplant");
	this->classes.push_back("sheep");
	this->classes.push_back("sofa");
	this->classes.push_back("train");
	this->classes.push_back("tvmonitor");
}

SSD::SSD(std::string prototxt, std::string model)
{
	this->prototxt = prototxt;
	this->caffemodel = model;
	this->net = readNetFromCaffe(this->prototxt, this->caffemodel);
	this->confidenceThreshold = 0.2;

	if (net.empty())
	{
		cerr << "Can't load network by using the following files: " << endl;
		cerr << "prototxt-file:     " << prototxt << endl;
		cerr << "caffeModel-file: " << model << endl;
		exit(-1);
	}

	this->classes.push_back("background");
	this->classes.push_back("aeroplane");
	this->classes.push_back("bicycle");
	this->classes.push_back("bird");
	this->classes.push_back("boat");
	this->classes.push_back("bottle");
	this->classes.push_back("bus");
	this->classes.push_back("car");
	this->classes.push_back("cat");
	this->classes.push_back("chair");
	this->classes.push_back("cow");
	this->classes.push_back("diningtable");
	this->classes.push_back("dog");
	this->classes.push_back("horse");
	this->classes.push_back("motorbike");
	this->classes.push_back("person");
	this->classes.push_back("pottedplant");
	this->classes.push_back("sheep");
	this->classes.push_back("sofa");
	this->classes.push_back("train");
	this->classes.push_back("tvmonitor");
}

vector<Rect> SSD::detectP(cv::Mat& frame)
{
	if (frame.empty())
	{
		waitKey();
		exit(-1);
	}

	Mat frame_cpy;
	resize(frame,frame_cpy,Size(300,300));
	Mat inputBlob = dnn::blobFromImage(frame_cpy, 0.007843, Size(300, 300), Scalar(127.5,127.5,127.5),false); //Convert Mat to batch of images
	this->net.setInput(inputBlob,"data");                   //set the network input
	Mat detection = net.forward("detection_out");   //compute output
	Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

	vector<Rect> result;
	for(int i = 0; i < detectionMat.rows; i++)
	{
		float confidence = detectionMat.at<float>(i, 2);
		int idx = static_cast<int>(detectionMat.at<float>(i, 1));
		if(confidence > confidenceThreshold && idx == 15)
		{
			int xLeftBottom = static_cast<int>(detectionMat.at<float>(i, 3) * frame.cols);
			int yLeftBottom = static_cast<int>(detectionMat.at<float>(i, 4) * frame.rows);
			int xRightTop = static_cast<int>(detectionMat.at<float>(i, 5) * frame.cols);
			int yRightTop = static_cast<int>(detectionMat.at<float>(i, 6) * frame.rows);

			Rect object((int)xLeftBottom, (int)yLeftBottom,
				(int)(xRightTop - xLeftBottom),
				(int)(yRightTop - yLeftBottom));

			result.push_back(object);
		}
	}

	return result;
}

