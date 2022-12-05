#include <module_pf/PathManager.hpp>

using namespace std;

const string PathManager::FOLLOWED_PERSON_NAME = "Leader";
const string PathManager::PAUSE = "StopFollowing";
const string PathManager::RESUME = "FollowPerson";

PathManager::PathManager(ros::NodeHandle nh_):
tf2_buffer_(),
tf2_l_(tf2_buffer_)
{
    //Bander para determinar si el robot se encuantre en momvimiento
    robot_is_static_ = true;
    first_pose_received_ = false;

	//Bandera del primer mensaje de tracking publicado como POI
	first_poi_sent_ = false;

	//Bandera para pausar al nodo
	pause_ = false;

	//Bandera para debugear
	nh_.param<bool>("/module_pf/debug",debug_,true);

	//Bandera para publicar el POI del la persona seguida y poder verlo con homer_gui
	//(propósito puramente de visualizacion, este nodo no requiere publicar dicho POI
	//para funcrionar)
	nh_.param<bool>("/module_pf/publish_person_poi",publish_person_poi_,false);

	//Tópico para girar la cabeza del robot
	string panhead_output_topic;
	nh_.param<string>("/module_pf/panhead_output_topic",panhead_output_topic,"/panhead/position");

	//Tópico para publicar el permiso de rotar la base
	string rotperm_topic;
	nh_.param<string>("/module_pf/control_rotperm_topic",rotperm_topic,"/module_pf/panhead/permission");

	//Tópicos para subscribirse a el resultado del tracking y a la pose del robot
	string track_output_topic;
	string robot_pose_topic;
	nh_.param<string>("/module_pf/track_output_topic",track_output_topic,"/module_pf/track_output");
	nh_.param<string>("/module_pf/pose_topic",robot_pose_topic,"/pose");

	//Tópicos para publicar commandos de navegación del robot
	string nav_start_topic;
	string nav_stop_topic;
	nh_.param<string>("/module_pf/nav_start_topic",nav_start_topic,"/module_pf/homer_navigation/start_navigation");
	nh_.param<string>("/module_pf/nav_stop_topic",nav_stop_topic,"/module_pf/homer_navigation/stop_navigation");

	//Tópicos para publicar commandos de navegación del robot
	string poi_add_topic;
	string poi_mod_topic;
	string poi_del_topic;
	nh_.param<string>("/module_pf/poi_add_topic",poi_add_topic,"/map_manager/add_POI");
	nh_.param<string>("/module_pf/poi_mod_topic",poi_mod_topic,"/map_manager/modify_POI");
	nh_.param<string>("/module_pf/poi_del_topic",poi_del_topic,"/map_manager/delete_POI");

	//Topicos de communicacion con el nodo MASTER
	string comm_output_topic;
	nh_.param<string>("/module_pf/master_out_topic",comm_output_topic,"/module_pf/comm_output");

	//Distancia minima entre POI actual y la nueva pos. del sujeto (dada en metros)
	//que debe existir para actualizar la posición de la persona en el mapa
	nh_.param<float>("/module_pf/poi_separation_thresh",poi_separation_thresh_,0.35);

	//Rango dentro del cual se considera que el robot ha alcanzado un POI del recorrido
	nh_.param<float>("/module_pf/poi_reached_thresh",poi_reached_thresh_,0.25);

	//Distancia dentro de la cual se considera que el robot ya está cerca de la persona
	//y por tanto no es necesario moverse
	nh_.param<float>("/module_pf/person_neig_range",person_neig_range_,0.5);

	//Rango dentro del cual se considera que el robot no se ha movido
	nh_.param<float>("/module_pf/not_moving_range",not_moving_range_,0.3);

	//Rango dentro del cual se considera que el robot no se ha movido
    int temp;
	nh_.param<int>("/module_pf/not_moving_period",temp,5000);
    not_moving_period_ = static_cast<unsigned int>(temp);

	//Links involucrados en el mapeo del sujeto al mapa
	nh_.param<string>("/module_pf/depthcam_link",depthcam_link_,"orbbec_astra_head_cam_depth_optical_frame");
	nh_.param<string>("/module_pf/person_link",person_link_,"followed_person");

	//Registro de los publicadores y subscriptores
	nav_start_pub_ = nh_.advertise<homer_mapnav_msgs::StartNavigation>(nav_start_topic,10);
	nav_stop_pub_ = nh_.advertise<homer_mapnav_msgs::StopNavigation>(nav_stop_topic,10);
	poi_add_pub_ = nh_.advertise<homer_mapnav_msgs::PointOfInterest>(poi_add_topic,10);
	poi_mod_pub_ = nh_.advertise<homer_mapnav_msgs::ModifyPOI>(poi_mod_topic,10);
	poi_del_pub_ = nh_.advertise<homer_mapnav_msgs::DeletePointOfInterest>(poi_del_topic,10);
	track_sub_ = nh_.subscribe(track_output_topic,10,&PathManager::trackCB,this);
	pose_sub_ = nh_.subscribe(robot_pose_topic,10,&PathManager::poseCB,this);
	comm_sub_ = nh_.subscribe(comm_output_topic,10,&PathManager::commCB,this);
	rotperm_pub_ = nh_.advertise<std_msgs::Bool>(rotperm_topic,10);

	//Inicialización del publicador al controlador de la cabeza
	panhead_pub_ = nh_.advertise<std_msgs::Float64>(panhead_output_topic,10);
}

void PathManager::trackCB(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	//Bandera para pausar al nodo
	if(pause_) return;

	//Un lock para evitar la lectura/escritura de/a una variable de manera simultanea
	if(!mutex_.try_lock()) return;

	//Leer datos de entrada
	geometry_msgs::Vector3 t_data;
	try
	{
		t_data = msg->vector;
	}
	catch(exception& e)
	{
		cout << e.what() << endl;
		mutex_.unlock();
		return;
	}

	if(debug_) cout << ">> Received track." << endl;

	//Obten una copia del valor mas reciente de la pose del robot
	geometry_msgs::Pose p_data = pose_data_;

	//Conversion del vector de pos. relativa -> posicion en el mapa
	homer_mapnav_msgs::PointOfInterest fp_poi;
	if(!vec2POI(t_data,p_data,fp_poi))
	{
		//No se pudo revisar la transformada (mapa) -> (sensor de profundidad)
		mutex_.unlock();
		return;
	}

	//Tomar acciones de acuerdo a:
	//	- posicion de la persona
	//	- posicion del robot
	//	- los POI previamente generados
	updatePathPerson(fp_poi,p_data);

	mutex_.unlock();
}

void PathManager::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	//Leer la pose del robot
	try
	{
		pose_data_ = msg->pose;

        if(first_pose_received_) isMoving(msg->header,msg->pose);
        else
        {
            pose_ref_ = msg->pose;
            head_ref_ = msg->header;
            first_pose_received_ = true;
        }
	}
	catch(exception& e)
	{
		cout << e.what() << endl;
	}

	return;
}

void PathManager::commCB(const std_msgs::String::ConstPtr& msg)
{
	if(msg->data == PathManager::PAUSE)
	{
		//Pausar al nodo
		pause_ = true;

		//Enviar comando para detener navegación
		homer_mapnav_msgs::StopNavigation stop_nav;
		nav_stop_pub_.publish(stop_nav);

		//Eliminar los POIs pendientes en el camino
		person_path_.clear();
	}
	else if(msg->data == PathManager::RESUME)
	{
		//Rehabilitar al nodo
		pause_ = false;
	}
}

tf::Vector3 PathManager::q2RPY(tf::Quaternion const &q)
{
	double roll,pitch,yaw;

	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q.getW() * q.getX() + q.getY() * q.getZ());
	double cosr_cosp = +1.0 - 2.0 * (q.getX() * q.getX() + q.getY() * q.getY());
	roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.getW() * q.getY() - q.getZ() * q.getX());
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q.getW() * q.getZ() + q.getX() * q.getY());
	double cosy_cosp = +1.0 - 2.0 * (q.getY() * q.getY() + q.getZ() * q.getZ());  
	yaw = atan2(siny_cosp, cosy_cosp);

	return tf::Vector3(roll,pitch,yaw);
}

bool PathManager::vec2POI(geometry_msgs::Vector3 const &vec,
			  geometry_msgs::Pose const &robot_pose,
			  homer_mapnav_msgs::PointOfInterest &poi)
{
	//Variables temporales
	tf::Vector3 trans;
	tf::Quaternion rot;

	//Construcción de la transformada (mapa) -> (base_footprint)
	tf::Transform m2bf_t;
	trans.setX(robot_pose.position.x);
	trans.setY(robot_pose.position.y);
	trans.setZ(robot_pose.position.z);
	rot = tf::Quaternion(robot_pose.orientation.x,
			     robot_pose.orientation.y,
			     robot_pose.orientation.z,
			     robot_pose.orientation.w);
	m2bf_t.setOrigin(trans);
	m2bf_t.setRotation(rot);

	//Construcción de la transformada (base_footprint) -> (base_link)
	tf::Transform bf2bl_t;
	trans.setX(0.0);
	trans.setY(0.0);
	trans.setZ(0.025);
	rot = tf::Quaternion(0.0,0.0,0.0,1.0);
	bf2bl_t.setOrigin(trans);
	bf2bl_t.setRotation(rot);

	//Consulta de la transformada (base_link) -> (sensor de profundidad)
	tf::Transform bl2ds_t;
	geometry_msgs::TransformStamped tf2_t;
	bool got_tf = false;
	for(unsigned int i = 0; i < 5; i++)
	{
		try
		{
			tf2_t = tf2_buffer_.lookupTransform("base_link",depthcam_link_,ros::Time(0));

			trans.setX(tf2_t.transform.translation.x);
			trans.setY(tf2_t.transform.translation.y);
			trans.setZ(tf2_t.transform.translation.z);
			rot = tf::Quaternion(tf2_t.transform.rotation.x,
					     tf2_t.transform.rotation.y,
					     tf2_t.transform.rotation.z,
					     tf2_t.transform.rotation.w);
			bl2ds_t.setOrigin(trans);
			bl2ds_t.setRotation(rot);

			got_tf = true;
		}
		catch(tf2::TransformException &ex)
		{
			ros::Duration(0.05).sleep();
			continue;
		}

		if(got_tf) break;
	}
	if(!got_tf) return false;

	//Ajuste del vector de posicion
	tf::Vector3 adj_vec;
	adj_vec.setX(vec.z);// Eje normal a la lente del sensor
	adj_vec.setY(vec.x * (-1));// Eje a lo largo de los laterales del sensor
	adj_vec.setZ(vec.y); // Eje normal al suelo

	//Construccion de la transformada (mapa) -> (sensor de profundidad)
	tf::Transform m2fp_t;
	m2fp_t = m2bf_t * bf2bl_t* bl2ds_t;// (mapa) -> (sensor de profundidad)

	//Transformación del centroide de la persona seguida en un POI dentro del mapa
	tf::Vector3 poi_vec = m2fp_t * adj_vec;
	poi.pose.position.x = poi_vec.getX();
	poi.pose.position.y = poi_vec.getY();
	poi.pose.position.z = poi_vec.getZ();
	poi.pose.orientation.x = 0.0;
	poi.pose.orientation.y = 0.0;
	poi.pose.orientation.z = 0.0;
	poi.pose.orientation.w = 1.0;
	poi.type = poi.PERSON;
	poi.name = FOLLOWED_PERSON_NAME;

	return true;
}

void PathManager::printV(tf::Vector3 const &v)
{
	cout << "Vector: " << v.getX() << ",";
	cout << v.getY() << ",";
	cout << v.getZ() << "\n";
	cout << endl;
}

void PathManager::updatePathPerson(homer_mapnav_msgs::PointOfInterest &fp_poi,
				   geometry_msgs::Pose &robot_pose)
{
	//Calcula la distancia entre POI de entrada y el almacenado
	float poi_dist = sqrt
	(
		pow(fp_poi.pose.position.x - current_person_poi_.pose.position.x,2) +
		pow(fp_poi.pose.position.y - current_person_poi_.pose.position.y,2)
	);

	//Calcula la distancia entre la posición de la persona y el robot
	float per2rob_dist = sqrt
	(
		pow(fp_poi.pose.position.x - robot_pose.position.x,2) +
		pow(fp_poi.pose.position.y - robot_pose.position.y,2)
	);

	//Publicación del POI de la persona seguida, para visualización
	if(!first_poi_sent_ && publish_person_poi_) //Se agrega el POI de la persona
	{
		first_poi_sent_ = true;
		poi_add_pub_.publish(fp_poi);
	}
	else if(publish_person_poi_ &&
		(poi_dist > poi_separation_thresh_))//Se actualize el POI de la persona
	{
		homer_mapnav_msgs::ModifyPOI modpoi;
		modpoi.poi = fp_poi;
		modpoi.old_name = fp_poi.name;
		poi_mod_pub_.publish(modpoi);
	}

	//El robot se encuentra dentro de la vecindad de cercanía
	if(per2rob_dist < person_neig_range_)
	{
		//Publica comando para detener la navegación
		homer_mapnav_msgs::StopNavigation stop_nav;
		nav_stop_pub_.publish(stop_nav);

		//Elimina el recorrido restante de la persona
		if(debug_) cout << "-------------------------------------------------" << endl;
		if(debug_) cout << ">> ROBOT WITHIN PERSON NEIGBORHOOD. STOPPING NAV." << endl;
		if(publish_person_poi_)
		{
			homer_mapnav_msgs::DeletePointOfInterest dpoi;
			for(unsigned int i = 0; i < person_path_.size(); i++)
			{
				stringstream ss;
				ss << i;
				dpoi.name = "p" + ss.str();
				poi_del_pub_.publish(dpoi);
			}
		}

		person_path_.clear();
		if(debug_) cout << ">> PERSON-PATH SIZE:" << person_path_.size() << endl;

        //Publica al nodo que controla la cabeza del robot permiso para girar la base
        std_msgs::Bool perm_msg;
        perm_msg.data = true;
        rotperm_pub_.publish(perm_msg);
	}
	//El robot se encuentra "lejos" de la persona y debe navegar para seguirla
	else
	{
        //Publica al nodo que controla la cabeza del robot no-permiso para girar la base
        std_msgs::Bool perm_msg;
        perm_msg.data = false;
        rotperm_pub_.publish(perm_msg);

		if(person_path_.size() == 0)
		{
			//Publica comando para comenzar a navegar
			homer_mapnav_msgs::StartNavigation start_nav;
			start_nav.goal = fp_poi.pose;
			start_nav.distance_to_target = 0.2;
			start_nav.skip_final_turn = true;
			start_nav.fast_planning = true;
			nav_start_pub_.publish(start_nav);

			//Almacena la posición a la que se está navegando
			person_path_.push_back(fp_poi.pose);
			if(publish_person_poi_)
			{
				homer_mapnav_msgs::PointOfInterest temp_p = fp_poi;
				temp_p.name = "p0";
				temp_p.type = temp_p.DEFAULT;
				poi_add_pub_.publish(temp_p);
			}
			if(debug_) cout << "-------------------------------------------------" << endl;
			if(debug_) cout << ">> ROBOT OUTSIDE PERSON NEIGBORHOOD. STARTING NAV." << endl;
			if(debug_) cout << ">> PERSON-PATH SIZE:" << person_path_.size() << endl;
		}
		else
		{
			//Calcula la distancia entre la pos. actual de la persona y la última pos. en person_path_
			geometry_msgs::Pose last_pathpoi = person_path_[person_path_.size()-1];
			float last_pathpoi_dist = sqrt
			(
				pow(fp_poi.pose.position.x - last_pathpoi.position.x,2) +
				pow(fp_poi.pose.position.y - last_pathpoi.position.y,2)
			);

			//Agrega un nuevo punto en el recorrido a navegar
			if(last_pathpoi_dist > poi_separation_thresh_)
			{
				person_path_.push_back(fp_poi.pose);
				if(publish_person_poi_)
				{
					homer_mapnav_msgs::PointOfInterest temp_p = fp_poi;
					stringstream ss;
					ss << person_path_.size() - 1;
					temp_p.name = "p" + ss.str();
					temp_p.type = temp_p.DEFAULT;
					poi_add_pub_.publish(temp_p);
				}
				if(debug_) cout << "-------------------------------------------------" << endl;
				if(debug_) cout << ">> PERSON IS MOVING. NEW POI ADDED TO PATH" << endl;
				if(debug_) cout << ">> PERSON-PATH SIZE:" << person_path_.size() << endl;
			}
		}
	}

	//Guarda la posicion mas reciente de la persona seguida
	current_person_poi_ = fp_poi;
}

void PathManager::updatePathRobot()
{
	//Si el recorrido de navegación está vacío no hay nada por hacer
	if(person_path_.size() == 0) return;

    //Publica al nodo que controla la cabeza del robot permiso para girar la base
    std_msgs::Bool perm_msg;
    perm_msg.data = false;
    rotperm_pub_.publish(perm_msg);

	//Calcula la distancia entre la pos. actual del robot y la primera pos. en person_path_
	geometry_msgs::Pose first_pathpoi = person_path_[0];
	geometry_msgs::Pose robot_pose = pose_data_;
	float first_pathpoi_dist = sqrt
	(
		pow(robot_pose.position.x - first_pathpoi.position.x,2) +
		pow(robot_pose.position.y - first_pathpoi.position.y,2)
	);

	//Determina si ya se ha alcanzado (aproximadamente) el POI al que se está navegando
	if(first_pathpoi_dist < poi_reached_thresh_)
	{
		if(debug_) cout << "-------------------------------------------------" << endl;
		if(debug_) cout << ">> ROBOT REACHED OLDEST POI IN PATH. DELETING OLDEST POI." << endl;
		if(publish_person_poi_)
		{
			homer_mapnav_msgs::DeletePointOfInterest dpoi;
			dpoi.name = "p0";
			poi_del_pub_.publish(dpoi);
		}
		//Solo queda un POI en el recorrido
		if(person_path_.size() == 1)
		{
            //Centra la cabeza del robot en caso de haber alcanzado el último POI de su cola, esto normalmente sucede
            //cuando el robot ha perdido de vista al operador
            centerRobotHead();

			if(debug_) cout << ">> STOPPING NAV." << endl;
			//Publica comando para detener la navegación
			homer_mapnav_msgs::StopNavigation stop_nav;
			nav_stop_pub_.publish(stop_nav);

			//Elimina el recorrido restante de la persona
			person_path_.clear();
		}
		//Quedan más POIs pendiente sen el recorrido
		else
		{
			if(debug_) cout << ">> NAVIGATING TO NEXT POI IN PATH." << endl;
			//Publicar comando para navegar al siguiente POI
			homer_mapnav_msgs::StartNavigation start_nav;
			start_nav.goal = person_path_[1];
			start_nav.distance_to_target = 0.2;
			start_nav.skip_final_turn = true;
			start_nav.fast_planning = true;
			nav_start_pub_.publish(start_nav);

			//Eliminar el POI alcanzado del recorrido
			person_path_.erase(person_path_.begin());
		}
		if(debug_) cout << ">> PERSON-PATH SIZE:" << person_path_.size() << endl;
	}
}

void PathManager::isMoving(std_msgs::Header h, geometry_msgs::Pose p)
{
    //Compute the distance between the current pose and the ref
    float dist = sqrt(
            pow((p.position.x - pose_ref_.position.x),2) + 
            pow((p.position.y - pose_ref_.position.y),2) );

    //Evaluate if the robot is within range
    if(dist < not_moving_range_)
    {
        //Compute the time elapsed since the las "reference pose" was defined
        unsigned int sec = h.stamp.sec;
        unsigned int nsec = h.stamp.nsec;
        unsigned int last_sec_ = head_ref_.stamp.sec;
        unsigned int last_nsec_ = head_ref_.stamp.nsec;
        stringstream ss1("");
        ss1 << sec;
        string s_sec = ss1.str();

        if(s_sec.length() > 6)
        {
            s_sec = s_sec.substr(s_sec.length()-6-1);
            sec = atoi(s_sec.c_str());
        }

        unsigned int lst_sec;
        stringstream ss2("");
        ss2 << last_sec_;
        string s_lst_sec = ss2.str();

        if(s_lst_sec.length() > 6)
        {
            s_lst_sec = s_lst_sec.substr(s_lst_sec.length()-6-1);
            lst_sec = atoi(s_lst_sec.c_str());
        }
        else lst_sec = last_sec_;

        unsigned int elapsed_ms = (sec*1000 + nsec/1000000) -
                                  (lst_sec*1000 + last_nsec_/1000000);

        //Evaluate if the required period has elapsed, in order to consider the robot is static
        if(elapsed_ms > not_moving_period_)
        {
            //if(!robot_is_static_) cout << "elapsed T: " << elapsed_ms << " | nmt: " << not_moving_period_ << endl;

            robot_is_static_ = true;
        }
    }
    else
    {
        //Reset the pose and timestamp references
        pose_ref_ = p;
        head_ref_ = h;

        //Also unset the "robot is static" flag
        robot_is_static_ = false;
    }
}

void PathManager::centerRobotHead()
{
    //Publica el angulo central como posicion deseada
    std_msgs::Float64 msg;
    msg.data = 0.0;
    panhead_pub_.publish(msg);
}
