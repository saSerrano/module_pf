#include <module_pf/TrackerNode.hpp>

using namespace std;
using namespace cv;

const string TrackerNode::FOLLOW_PERSON = "FollowPerson";
const string TrackerNode::STOP_FOLLOWING = "StopFollowing";

TrackerNode::TrackerNode(ros::NodeHandle nh_,
			 std::string color_img_topic,
			 std::string depth_img_topic):
it_(nh_),
dsub_(it_,depth_img_topic,1),
csub_(it_,color_img_topic,1),
sync_(mySyncPolicy(15),dsub_,csub_),
state_(TrackerNode::TRACKER_STATE_IDLE),
prev_state_(TrackerNode::TRACKER_STATE_IDLE),
followed_person_id_(-1)
{
	//Llenado del vector de colores que son utilizados para
	//mostrar el resutado de la segmentacion
	colorTable_.clear();
	colorTable_.push_back(cv::Scalar(153,153,255));
	colorTable_.push_back(cv::Scalar(153,204,255));
	colorTable_.push_back(cv::Scalar(153,255,255));
	colorTable_.push_back(cv::Scalar(153,255,153));
	colorTable_.push_back(cv::Scalar(255,255,153));
	colorTable_.push_back(cv::Scalar(255,153,153));
	colorTable_.push_back(cv::Scalar(255,153,255));
	colorTable_.push_back(cv::Scalar(0,0,255));
	colorTable_.push_back(cv::Scalar(0,128,255));
	colorTable_.push_back(cv::Scalar(0,255,255));
	colorTable_.push_back(cv::Scalar(0,255,0));
	colorTable_.push_back(cv::Scalar(255,255,0));
	colorTable_.push_back(cv::Scalar(255,0,0));
	colorTable_.push_back(cv::Scalar(255,0,255));
	colorTable_.push_back(cv::Scalar(0,0,102));
	colorTable_.push_back(cv::Scalar(0,51,102));
	colorTable_.push_back(cv::Scalar(0,102,102));
	colorTable_.push_back(cv::Scalar(0,102,0));
	colorTable_.push_back(cv::Scalar(102,102,0));
	colorTable_.push_back(cv::Scalar(102,0,0));
	colorTable_.push_back(cv::Scalar(102,0,102));

	//Tópico para girar la cabeza del robot
	string panhead_output_topic;
	nh_.param<string>("/module_pf/panhead_output_topic",panhead_output_topic,"/panhead/position");

	//Topicos de communicacion con el nodo MASTER
	string comm_input_topic,comm_output_topic;
	nh_.param<string>("/module_pf/master_in_topic",comm_input_topic,"/module_pf/comm_input");
	nh_.param<string>("/module_pf/master_out_topic",comm_output_topic,"/module_pf/comm_output");

	//Topico para la publicacion del la posicion del sujeto siendo seguido
	string track_output_topic;
	nh_.param<string>("/module_pf/track_output_topic",track_output_topic,"/module_pf/track_output");

	//Tolerancia de segmentacion (dada en metros)
	float st_;
	nh_.param<float>("/module_pf/segmentation_thresh",st_,0.05);

	//Tolerancia de tracking (dada en metros)
	float dt_;
	nh_.param<float>("/module_pf/track_dist_thresh",dt_,0.3);

	//Umbral de pixeles que debe tener un objeto para tomarlo en cuenta
	int sa_;
	nh_.param<int>("/module_pf/significant_area",sa_,4000);

	//Bandera para saber si se muestra una imagen resultante del tracking
	nh_.param<bool>("/module_pf/display_image_flag",display_image_flag_,true);

	//Umbral de valor minimo para el traslape entre BB de personas y TrackedObject
	nh_.param<float>("/module_pf/overlap_thresh",overlap_thresh_,0.8);

	//Bandera para mover o no la cabeza del robot
	nh_.param<bool>("/module_pf/move_head",move_head_,true);

	//Registro del metodo 'syncCB'
	sync_.registerCallback(boost::bind(&TrackerNode::syncCB,this,_1,_2));

	//Inicialización del publicador de la posicion del sujeto
	track_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(track_output_topic,10);

	//Inicializacion del publicador y subscriptor de comunicacion con MASTER
	comm_sub_ = nh_.subscribe(comm_output_topic,10,&TrackerNode::commCB,this);
	comm_pub_ = nh_.advertise<std_msgs::String>(comm_input_topic,10);

	//Inicialización del publicador al controlador de la cabeza
	panhead_pub_ = nh_.advertise<std_msgs::Float64>(panhead_output_topic,10);

	//Inicialización de los objetos con las que se realiza el tracking
	tracker_ = Tracker(st_,dt_,sa_);
	prev_object_ = vector<TrackedObject>();

	//Inicialización del detector de personas
	std::string pkg_str = ros::package::getPath("module_pf");
	std::string prototxt(pkg_str+"/ssd/MobileNetSSD_deploy.prototxt.txt");
	std::string caffemodel(pkg_str+"/ssd/MobileNetSSD_deploy.caffemodel");
	ssd_ = SSD(prototxt,caffemodel);
}

TrackerNode::~TrackerNode()
{
	destroyAllWindows();
}

void TrackerNode::syncCB(const sensor_msgs::ImageConstPtr& d_msg,const sensor_msgs::ImageConstPtr& c_msg)
{
	//Se usa un 'lock' para asegurar que las variables de la clase estan siendo accesadas
	//por un unico hilo simultaneamente
	if(!mutex_.try_lock()) return;

	//Obtencion de la imagen de color y profundidad en forma de un objeto 'Mat'.
	cv_bridge::CvImagePtr d_ptr,c_ptr;
	Mat img,c_img;
	try
	{
		d_ptr = cv_bridge::toCvCopy(d_msg);
		c_ptr = cv_bridge::toCvCopy(c_msg,"bgr8");
		img = d_ptr->image;
		c_img = c_ptr->image;
		/*
		NOTA: Los pixeles de la imagen de profundidad tienen precision float,
		      donde los valores de los pixeles representan la distancia, a lo
		      largo del Z, dada en metros.
		*/
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		mutex_.unlock();
		return;
	}

	//Se reduce el tamaño de la imagen de profundidad a la mitad, para reducir el
	//costo computacional del proceso de segmentacion
	resize(img,img,Size(img.cols/SCALE_FACTOR,img.rows/SCALE_FACTOR));

	//Extrae los objetos apartir de la imagen de profundidad
	std::vector<TrackedObject> currentObjects;
	currentObjects = tracker_.extractObjects(img);

	//Detección de personas
	std::vector<Rect> currentPeople = ssd_.detectP(c_img);

	//Vincula los objetos extraidos en el instante actual con los del instante
	//previo para asignarles su respectiva ID
	tracker_.linkObjects(currentObjects,prev_object_);

	//Guarda los objetos actuales para el siguiente instante
	prev_object_ = currentObjects;

	//Mover la cabeza del robot para mantener dentro del rango de visión a la persona seguida
	if(move_head_ && state_ == TrackerNode::TRACKER_STATE_FOLLOWING) moveRobotHead(currentObjects,true);

	//Dependiendo el estado del tracker, son las acciones que debe ejecutar
	switch(state_)
	{
		//Identificar a la persona a seguir y publicar su posición relativa al robot
		case TrackerNode::TRACKER_STATE_FOLLOWING:
		{
			following(currentObjects,currentPeople,c_msg->header);
		}
		break;

		//Recuperar la persona que estaba siendo seguida
		case TrackerNode::TRACKER_STATE_FINDING_FOLLOWED_PERSON:
		{
			searching(currentObjects,currentPeople,c_msg->header);
		}
		break;

		default:
		break;
	}

	//Despliegue de la imagen resultante de la segmentacion y el seguimiento
	if(display_image_flag_) displayInfo(c_img,img,currentObjects,currentPeople);
		
	mutex_.unlock();
	return;
}

void TrackerNode::commCB(const std_msgs::String::ConstPtr& msg)
{
	if(state_ == TrackerNode::TRACKER_STATE_IDLE && msg->data == TrackerNode::FOLLOW_PERSON)
	{
		prev_state_ = state_;
		state_ = TrackerNode::TRACKER_STATE_FINDING_FOLLOWED_PERSON;
	}
	else if(msg->data == TrackerNode::STOP_FOLLOWING)
	{
		prev_state_ = state_;
		state_ = TrackerNode::TRACKER_STATE_IDLE;

        // Prevent the head from keep moving
        vector<TrackedObject> tmp;
        moveRobotHead(tmp,false);
	}
}

void TrackerNode::displayInfo(Mat &c_img,Mat &d_img,std::vector<TrackedObject> const &currentObjects, std::vector<Rect> const &currentPeople)
{
	//Colorear los objetos segmentados, usando los colores almacenados
	//en el vector 'colorTable_'
	Mat mask = Mat::zeros(d_img.rows+2,d_img.cols+2,CV_8UC1);
	Rect roi(1,1,d_img.cols,d_img.rows);
	Mat color_img = Mat::zeros(d_img.rows,d_img.cols,CV_8UC3);
	for(unsigned int i = 0; i < currentObjects.size(); i++)
	{
		mask.setTo(Scalar(0));
		floodFill(d_img,mask,currentObjects[i].seedPoint,Scalar(0.0f),NULL,Scalar(tracker_.sThresh()),Scalar(tracker_.dThresh()),8 | (255 << 8) | FLOODFILL_MASK_ONLY);
		color_img.setTo(colorTable_[i],mask(roi));
	}

	//Mostrar el ID de cada objeto, en el centro de su bounding-box
	for(unsigned int i = 0; i < currentObjects.size(); i++)
	{
		Rect person_roi = correctBox(color_img,currentObjects[i].boundingBox);
		rectangle(color_img,person_roi,Scalar(0,255,0),2);
		Point pp_;
		pp_.x = (currentObjects[i].boundingBox.x + (currentObjects[i].boundingBox.width / 2));
		pp_.y = (currentObjects[i].boundingBox.y + (currentObjects[i].boundingBox.height / 2));
		stringstream ss("");
		ss << currentObjects[i].id;
		putText(color_img,ss.str(),pp_,FONT_HERSHEY_SIMPLEX,1.0,Scalar(0,0,255));
	}

	//Regresar la imagen a sus dimensines originales
	resize(color_img,color_img,Size(d_img.cols*SCALE_FACTOR,d_img.rows*SCALE_FACTOR));

	//Identificar el BB de la persona que está siendo seguida
	int fp_index = -1;
	for(unsigned int i = 0; i < currentObjects.size(); i++)
	{
		if(currentObjects[i].id == followed_person_id_)
		{
			TrackedObject t_o = currentObjects[i];
			t_o = t_o.scale(SCALE_FACTOR);
			float max_overlap = 0.0;
			for(unsigned int j = 0; j < currentPeople.size(); j++)
			{
				float temp_overlap = TrackedObject::overlap(t_o.boundingBox,currentPeople[j]);
				if(temp_overlap > max_overlap && temp_overlap > overlap_thresh_)
				{
					max_overlap = temp_overlap;
					fp_index = j;
				}
			}

			break;
		}
	}

	//Mostrar las detecciones de personas en la imagen de color
	for(unsigned int i = 0; i < currentPeople.size(); i++)
	{
		if(i == fp_index) continue;
		Rect person_roi = correctBox(c_img,currentPeople[i]);
		rectangle(c_img,person_roi,Scalar(255,0,0),2);
	}
	//El BB de la persona seguida se dibuja al final en color rojo
	if(fp_index != -1)
	{
		Rect person_roi = correctBox(c_img,currentPeople[fp_index]);
		rectangle(c_img,person_roi,Scalar(0,0,255),2);
	}

	//Mostrar el estado en el que se encuentra el tracker
	string t_state("State: ");
	switch(state_)
	{
		case TrackerNode::TRACKER_STATE_IDLE:
			t_state += "Idle";
		break;

		case TrackerNode::TRACKER_STATE_FOLLOWING:
			t_state += "Following";
		break;

		case TrackerNode::TRACKER_STATE_FINDING_FOLLOWED_PERSON:
			t_state += "Searching";
		break;
	}
	putText(c_img,t_state,cv::Point(10,60),FONT_HERSHEY_PLAIN,1.25,Scalar(0,255,0),2);

	//Unir la imagen de deteccion de objetos y la de detección de personas en una sola
	Mat dc_img(c_img.rows,c_img.cols*2,CV_8UC3);
	color_img.copyTo(dc_img(Rect(0,0,c_img.cols,c_img.rows)));
	c_img.copyTo(dc_img(Rect(c_img.cols,0,c_img.cols,c_img.rows)));

	//Despliegue de la imagen resultante del proceso de segmentacion
	imshow("Output Image",dc_img);
	waitKey(5);
}

cv::Rect TrackerNode::correctBox(cv::Mat const &img,cv::Rect const &r)
{
	Rect corr_r;

	corr_r.x = std::max(r.x,0);
	corr_r.y = std::max(r.y,0);

	corr_r.x = std::min(corr_r.x,img.cols-1);
	corr_r.y = std::min(corr_r.y,img.rows-1);

	corr_r.width = std::max(r.width,0);
	corr_r.height = std::max(r.height,0);

	corr_r.width = std::min(corr_r.width, img.cols - corr_r.x);
	corr_r.height = std::min(corr_r.height, img.rows - corr_r.y);

	return corr_r;
}

void TrackerNode::sendTrack(TrackedObject const &tracked_o,std_msgs::Header header)
{
	geometry_msgs::Vector3Stamped vec;
	vec.header = header;
	vec.vector.x = tracked_o.centroid.x;
	vec.vector.y = tracked_o.centroid.y;
	vec.vector.z = tracked_o.centroid.z;

	track_pub_.publish(vec);
}

void TrackerNode::moveRobotHead(std::vector<TrackedObject> const &cur_obj, bool use_ref)
{
	//Ignora la referencia y centra la cabeza del robot
	if(!use_ref)
	{
		//Publica la posicion deseada
		std_msgs::Float64 msg;
		msg.data = 0.0;
		panhead_pub_.publish(msg);

		return;
	}

	int leader_index(-1);
	for(unsigned int i = 0; i < cur_obj.size(); i++)
	{
		if(cur_obj[i].id == followed_person_id_)
		{
			leader_index = static_cast<int>(i);
			break;
		}
	}

	if(leader_index == -1) return;

	//Calcula el ángulo del vector con respecto al centro de la imagen
	float x_dist = cur_obj[leader_index].centroid.x;
	float z_dist = cur_obj[leader_index].centroid.z;
	float alpha = atan(x_dist / z_dist) * (-1.0);

	//Publica la posicion deseada
	std_msgs::Float64 msg;
	msg.data = alpha;
	panhead_pub_.publish(msg);
}

void TrackerNode::following(std::vector<TrackedObject> &o_vec,std::vector<Rect> &p_vec,std_msgs::Header header)
{
	//Verificar si se encontró al sujeto mediante la imagen de profundidad
	int subject_idx = -1;
	for(unsigned int i = 0; i < o_vec.size(); i++)
	{
		if(o_vec[i].id == followed_person_id_)
		{
			subject_idx = i;
			break;
		}
	}

	//Si se encontro la persona, re-evaluar si cumple el criterio de overlap
	if(subject_idx != -1)
    {
        TrackedObject tmp = o_vec[subject_idx].scale(SCALE_FACTOR);
        double max_overlap(0.0);
        for(unsigned int i = 0; i < p_vec.size(); i++)
        {
            double ol = TrackedObject::overlap(p_vec[i],tmp.boundingBox);
            if(ol > max_overlap) max_overlap = ol;
        }
    }

	//Se encontró a la persona con la info de profundidad
	if(subject_idx != -1)
	{
		//Publicar la posición de la persona (relativa al robot)
		sendTrack(o_vec[subject_idx],header);
	}
	//Se perdió al sujeto
	else
	{
		//Se transita al estado de búsqueda del sujeto
		prev_state_ = state_;
		state_ = TrackerNode::TRACKER_STATE_FINDING_FOLLOWED_PERSON;

		// Publicar el evento
		std_msgs::String event;
		event.data = "lost_followed_person";
		comm_pub_.publish(event);
	}
}

void TrackerNode::searching(std::vector<TrackedObject> &o_vec,std::vector<Rect> &p_vec,std_msgs::Header header)
{
	// Sort tracked objects: from closest to furthest
	vector<TrackedObject> sorted_o_vec(o_vec);
	std::sort(sorted_o_vec.begin(), sorted_o_vec.end(), TrackedObject::closer);

	// Compute the highest overlap of each blob
	int subject_idx = -1;
    vector<double> ol_score;
	for(unsigned int i = 0; i < sorted_o_vec.size(); i++)
	{
		TrackedObject tmp = sorted_o_vec[i].scale(SCALE_FACTOR);
        double max_overlap(0.0);
		for(unsigned int j = 0; j < p_vec.size(); j++)
		{
            double ol = TrackedObject::overlap(p_vec[j],tmp.boundingBox);
            if(ol > max_overlap) max_overlap = ol;
		}
        ol_score.push_back(max_overlap);

		if(subject_idx != -1) break;
	}

    // Gather the object with the highest overlap score
    double max_score(0.0);
    int max_index(-1);
    for(unsigned int i = 0; i < ol_score.size(); i++)
    {
        if(ol_score[i] > max_score)
        {
            max_score = ol_score[i];
            max_index = i;
        }
    }

    // Check if the object with the highest score is over the threshold
    if(ol_score[max_index] > overlap_thresh_) subject_idx = max_index;

	// The closest person to the robot was found
	if(subject_idx != -1)
	{
		//Recupera el ID en caso de no tenerlo
		followed_person_id_ = sorted_o_vec[subject_idx].id;

		//Publicar la posición de la persona (relativa al robot)
		sendTrack(sorted_o_vec[subject_idx],header);

		// Update the tracker's state
		prev_state_ = state_;
		state_ = TrackerNode::TRACKER_STATE_FOLLOWING;

		// Publicar el evento
		std_msgs::String event;
		event.data = "found_person_to_follow";
		comm_pub_.publish(event);
	}
}



















