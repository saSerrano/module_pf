#include <module_pf/Tracker.hpp>

TrackerPF::TrackerPF()
{
	Segmentation_Threshold = 0.05;
	Distance_Threshold_ = 0.3;
	significant_area = 4000;
	idRecord = 0;
}

TrackerPF::TrackerPF(float st, float dt, int sa)
{
	if(st >= 0 && dt >= 0 && sa >= 0)
	{
		Segmentation_Threshold = st;
		Distance_Threshold_ = dt;
		significant_area = sa;
	}
	else
	{
		Segmentation_Threshold = 0.05;
		Distance_Threshold_ = 0.3;
		significant_area = 4000;
	}

	idRecord = 0;
}

vector<TrackedObject> TrackerPF::extractObjects(Mat &img)
{
	//Segmentacion de los objetos con un area de por lo menos 'significant_area'
	Mat mask = Mat::zeros(img.rows+2,img.cols+2,CV_8UC1);
	Mat label = Mat::zeros(img.rows+2,img.cols+2,CV_8UC1);
	Mat mask2 = Mat::zeros(img.rows,img.cols,CV_8UC1);
	uchar significant_count = 0;
	vector<Point> significant_points;
	for(int j = 0; j < img.rows; j++)
	{
		for(int i = 0; i < img.cols; i++)
		{
			if(img.at<float>(j,i)!=0 && mask.at<uchar>(j+1,i+1)==0)
			{
				int area = floodFill(img,mask,Point(i,j),Scalar(0.0f),NULL,Scalar(Segmentation_Threshold),Scalar(Segmentation_Threshold),8 | (255 << 8) | FLOODFILL_MASK_ONLY);
				if(area > significant_area)
				{
					significant_count+=1;
					significant_points.push_back(Point(i,j));
					floodFill(img,label,Point(i,j),Scalar(0.0f),NULL,Scalar(Segmentation_Threshold),Scalar(Segmentation_Threshold),8 | (significant_count << 8) | FLOODFILL_MASK_ONLY);
				}
			}
		}
	}

	//Colecta de informacion espacial para calcular los centroides y bounding boxes
	vector<int> max_x(significant_count,0);
	vector<int> min_x(significant_count,99999);
	vector<int> max_y(significant_count,0);
	vector<int> min_y(significant_count,99999);
	vector<int> m00(significant_count,0);
	vector<float> m10(significant_count,0);
	vector<float> m01(significant_count,0);
	vector<float> m001(significant_count,0);
	for(int j = 1; j < label.rows-1; j++)
	{
		for(int i = 1; i < label.cols-1; i++)
		{
			uchar id = label.at<uchar>(j,i);
			if(id!=0)
			{
				id-=1;
				if(i-1 > max_x[id]) max_x[id] = i-1;
				if(i-1 < min_x[id]) min_x[id] = i-1;
				if(j-1 > max_y[id]) max_y[id] = j-1;
				if(j-1 < min_y[id]) min_y[id] = j-1;
				m00[id]++;
				m10[id] += static_cast<float>(i-1);
				m01[id] += static_cast<float>(j-1);
				m001[id] += img.at<float>(j-1,i-1);
			}
		}
	}

	//Calculo de los centroides y los bounding boxes
	vector<TrackedObject> currObj(significant_count,TrackedObject());
	for(unsigned int i = 0; i < currObj.size(); i++)
	{
		Point3f cp_;
		Rect bb_;
		cp_.z = m001[i]/m00[i];
		cp_.x = ((m10[i]/m00[i] - static_cast<float>(img.cols)/2.0)/static_cast<float>(img.cols)) * tan(30.0 * PI / 180.0) * cp_.z * 2;
		cp_.y = ((static_cast<float>(img.rows)/2.0 - m01[i]/m00[i])/static_cast<float>(img.rows)) * tan(24.75 * PI / 180.0) * cp_.z * 2;
		bb_.x = min_x[i];
		bb_.y = min_y[i];
		bb_.width = max_x[i] - min_x[i];
		bb_.height = max_y[i] - min_y[i];

		currObj[i].set(cp_,bb_,significant_points[i]);
	}

	return currObj;
}

void TrackerPF::linkObjects(vector<TrackedObject> &currObj, vector<TrackedObject> &prevObj)
{
	//Vincular los objetos con area significativa del instante actual
	//con los del instante previo
	if(prevObj.size() > 0 && currObj.size() > 0)
	{
		//Calcular la distancia de los objectos actuales con cada uno de
		//los del instante previo
		vector< vector<float> > cd_vec;//distancias entre el obj-actual con cada obj-previo
		vector<int> cid_vec;//el index del obj-previo mas cercano al obj-actual
		for(unsigned int i = 0; i < currObj.size(); i++)
		{
			vector<float> temp_cd_vec;
			float objMinDist = 999999;
			int closestIndex = -1;
			for(unsigned int j = 0; j < prevObj.size(); j++)
			{
				float temp_dist = currObj[i].fullEucD(prevObj[j]);
				if(temp_dist < objMinDist)
				{
					objMinDist = temp_dist;
					closestIndex = j;
				}

				temp_cd_vec.push_back(temp_dist);
			}

			cd_vec.push_back(temp_cd_vec);
			if(objMinDist <= Distance_Threshold_) cid_vec.push_back(closestIndex);
			else cid_vec.push_back(-1);
		}

		//Verificar que, a lo mucho, 1 objeto actual se asocie con cada objeto previo
		//y que sea el mas cercano
		for(unsigned int i = 0; i < prevObj.size(); i++)
		{
			int curr2prev_closest_index = -1;
			float curr2prev_mindist = 999999;
			for(unsigned int j = 0; j < cid_vec.size(); j++)
			{
				//Competidores
				if(cid_vec[j] != -1 && cid_vec[j] == i)
				{
					//El competidor mas cercano hasta el momento
					if(cd_vec[j][i] < curr2prev_mindist)
					{
						curr2prev_mindist = cd_vec[j][i];
						curr2prev_closest_index = j;
					}
				}
			}

			if(curr2prev_closest_index != -1)
			{
				for(unsigned int j = 0; j < cid_vec.size(); j++)
				{
					//Los competidores que perdieron seran reasignados, si es posible
					if(cid_vec[j] == i && curr2prev_closest_index != j)
					{
						float mindist = 999999;
						int c_index = -1;
						for(unsigned int k = i; k < prevObj.size(); k++)
						{
							if(k == i) continue;

							if(cd_vec[j][k] < mindist &&
							   cd_vec[j][k] < Distance_Threshold_)
							{
								mindist = cd_vec[j][k];
								c_index = k;
							}
						}

						cid_vec[j] = c_index;
					}
				}
			}
		}

		//Asignacion de IDs a objetos actuales
		for(unsigned int i = 0; i < currObj.size(); i++)
		{
			//Recupera el ID del objeto previo al que se asocio
			if(cid_vec[i] != -1) currObj[i].id = prevObj[cid_vec[i]].id;
			else
			{
				//No se le asocio con un objeto previo, se le asigna un
				//nuevo ID
				currObj[i].id = idRecord;
				idRecord++;
			}
		}
	}
	else if(currObj.size() > 0)
	{
		//Ya que no habia objetos en el instante previo con los cuales vincular
		//a los actuales, a todos los actuales se les proporciona un nuevo ID
		for(unsigned int i = 0; i < currObj.size(); i++)
		{
			currObj[i].id = idRecord;
			idRecord++;
		}
	}
}

float TrackerPF::sThresh()
{
	return Segmentation_Threshold;
}

float TrackerPF::dThresh()
{
	return Distance_Threshold_;
}
