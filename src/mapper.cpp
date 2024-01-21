
/*****************************************************/
//Organization: Stuba Green Team
//Authors: Martin Lučan, Patrik Knaperek, Filip Botka
/*****************************************************/

#include "../include/mapper.h"

Mapper::Mapper(ros::NodeHandle& nh)
{
  /* Load parameters */
  Utils::loadParam(nh, "euclid_threshold", 0.3f, &params_.euclid_th_);
  
  /* ROS interface initialization */
  pub_car_pose_ = nh.advertise<sgtdv_msgs::CarPose>("slam/pose", 1);
  pub_map_ = nh.advertise<sgtdv_msgs::ConeArr>("slam/map", 1);

  car_pose_sub_ = nh.subscribe("pose_estimate", 1, &Mapper::carPoseCallback, this);
  cones_sub_ = nh.subscribe("fusion_cones", 1, &Mapper::conesCallback, this);
  // cones_sub_ = nh.subscribe("fssim/camera/cones", 1, &Mapper::conesCallbackSim, this);
}

void Mapper::carPoseCallback(const sgtdv_msgs::CarPose::ConstPtr& msg)
{  
  sgtdv_msgs::CarPose carPose;
  carPose.position.x = msg->position.x;
  carPose.position.y = msg->position.y;
  carPose.yaw = msg->yaw;

  pub_car_pose_.publish(carPose);
}

void Mapper::conesCallback(const sgtdv_msgs::ConeStampedArr::ConstPtr& msg)
{
  geometry_msgs::PointStamped coordsBase, coordsMap;
  double newColor;

  for (const auto& cone : msg->cones)
    {
      if(cone.color == 'b'){newColor = 1;}   // 'b' = blue
      if(cone.color == 'y'){newColor = 2;}   // 'y' = yellow
      if(cone.color == 's' || cone.color == 'g'){newColor = 3;}   // 's' = orange small; 'g' = orange big

      coordsBase.header = cone.coords.header;
      coordsBase.point.x = cone.coords.x;
      coordsBase.point.y = cone.coords.y;

      try
      {
        listener_.transformPoint("map", coordsBase, coordsMap);
      }
      catch (tf::TransformException &e)
      {
        ROS_WARN_STREAM(e.what());
        continue;
      }

      dataAssEuclid(coordsMap.point.x, coordsMap.point.y, newColor);
    }
  
  pubCones();
}

void Mapper::conesCallbackSim(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  float const *temp;
  double newColor;

  for (int i = 0; i < msg->width; i++)
    {

      temp = reinterpret_cast<const float*>(&msg->data[i*msg->point_step]);
      geometry_msgs::PointStamped coordsBase, coordsMap;

      coordsBase.header = msg->header;
      coordsBase.point.x = *temp;
      coordsBase.point.y = *(temp + 1);

      if(*(temp + 9) > 0.85){newColor = 1;}   // 1 = blue
      if(*(temp + 10) > 0.85){newColor = 2;}   // 2 = yellow
      if(*(temp + 11) > 0.85){newColor = 3;}   // 3 = orange

      try
      {
        listener_.transformPoint("map", coordsBase, coordsMap);
      }
      catch (tf::TransformException &e)
      {
        ROS_WARN_STREAM(e.what());
        continue;
      }

      dataAssEuclid(coordsMap.point.x, coordsMap.point.y, newColor);
    }
  
  pubCones();
} 


void Mapper::dataAssEuclid(const double newX, const double newY, const double newColor)
{
  
  std::vector<double> newRow;
  std::vector<double> euclidVect;  
 
  if(cone_map_.empty() == true){
    newRow = {newX, newY, newColor, 1};
    cone_map_.push_back(newRow);
  } 
  else{
   
    
    for (const auto& cone : cone_map_)
    {
      const double euclidDist = sqrt(pow(newX - cone[0], 2) + pow(newY - cone[1], 2));
      euclidVect.emplace_back(euclidDist);
    }

    int minElementIndex = std::min_element(euclidVect.begin(), euclidVect.end()) - euclidVect.begin();

    if(euclidVect[minElementIndex] < params_.euclid_th_)
    {
      cone_map_[minElementIndex][0] = newX;
      cone_map_[minElementIndex][1] = newY;
      
      /* color decision */
      if (cone_map_[minElementIndex][2] == newColor)
      {
        cone_map_[minElementIndex][3]++;
      }
      else
      {
        if (cone_map_[minElementIndex][3] > 0)
        {
          cone_map_[minElementIndex][3] -= 5;
        }
        else
        {
          cone_map_[minElementIndex][2] = newColor;
          cone_map_[minElementIndex][3] = 1;
        }
      }
    }
    else
    {
      newRow = {newX, newY, newColor, 1};
      cone_map_.push_back(newRow); 
    }
  }
}

void Mapper::pubCones()
{
  std::vector<std::vector<double>>::iterator iter;
  static sgtdv_msgs::ConeArr coneArr;
  coneArr.cones.clear();

  sgtdv_msgs::Cone cone;
  for (iter = cone_map_.begin(); iter < cone_map_.end(); ++iter)
  {
    cone.coords.x = cone_map_[iter-cone_map_.begin()][0];
    cone.coords.y = cone_map_[iter-cone_map_.begin()][1];
    cone.color = cone_map_[iter-cone_map_.begin()][2];
    coneArr.cones.push_back(cone); 
  }

  pub_map_.publish(coneArr);
}