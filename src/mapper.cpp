
#include "../include/mapper/mapper.h"

using namespace std;

Mapper::Mapper()
{
}

  void Mapper::carPoseCallback(const sgtdv_msgs::CarPose::ConstPtr& msg)
{  
  sgtdv_msgs::CarPose carPose;
  carPose.position.x = msg->position.x;
  carPose.position.y = msg->position.y;
  carPose.yaw = msg->yaw;

  pubCarPose.publish(carPose);
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
        m_listener.transformPoint("map", coordsBase, coordsMap);
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
        m_listener.transformPoint("map", coordsBase, coordsMap);
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
  
  vector<double> newRow;
  vector<double> euclidVect;  
 
  if(m_coneMap.empty() == true){
    newRow = {newX, newY, newColor, 1};
    m_coneMap.push_back(newRow);
  } 
  else{
   
    
    for(int i=0; i < m_coneMap.size(); i++)
    {
      const double euclidDist = sqrt(pow(newX - m_coneMap[i][0], 2) + pow(newY - m_coneMap[i][1], 2));
      euclidVect.emplace_back(euclidDist);
    }

    int minElementIndex = std::min_element(euclidVect.begin(),euclidVect.end()) - euclidVect.begin();

    if(euclidVect[minElementIndex] < euclidThresh){
      m_coneMap[minElementIndex][0] = newX;
      m_coneMap[minElementIndex][1] = newY;
      
      /* color decision */
      if (m_coneMap[minElementIndex][2] == newColor)
      {
        m_coneMap[minElementIndex][3]++;
      }
      else
      {
        if (m_coneMap[minElementIndex][3] > 0)
        {
          m_coneMap[minElementIndex][3] -= 5;
        }
        else
        {
          m_coneMap[minElementIndex][2] = newColor;
          m_coneMap[minElementIndex][3] = 1;
        }
      }
    }
    else{
      newRow = {newX, newY, newColor, 1};
      m_coneMap.push_back(newRow); 
    }
  }
}

void Mapper::pubCones(){

  vector<vector<double>>::iterator iter;
  static sgtdv_msgs::ConeArr coneArr;
  coneArr.cones.clear();

  sgtdv_msgs::Cone cone;
  for(iter = m_coneMap.begin(); iter < m_coneMap.end(); ++iter)
  {
    cone.coords.x = m_coneMap[iter-m_coneMap.begin()][0];
    cone.coords.y = m_coneMap[iter-m_coneMap.begin()][1];
    cone.color = m_coneMap[iter-m_coneMap.begin()][2];
    coneArr.cones.push_back(cone); 
  }

  // ROS_INFO_STREAM("process cones duration: " << ros::Time::now().toSec() - m_processTimeStart.toSec());
  pubMap.publish(coneArr);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapper");
  ros::NodeHandle nh;

  Mapper mapper; 
  
  nh.getParam("euclid_threshold", mapper.euclidThresh);
  ros::Subscriber carStateSubReal = nh.subscribe("pose_estimate", 1, &Mapper::carPoseCallback, &mapper);
  ros::Subscriber conesSubReal = nh.subscribe("fusion_cones", 1, &Mapper::conesCallback, &mapper);
  // ros::Subscriber conesSubSim = nh.subscribe("fssim/camera/cones", 1, &Mapper::conesCallbackSim, &mapper);
  mapper.pubCarPose = nh.advertise<sgtdv_msgs::CarPose>("slam/pose", 1);
  mapper.pubMap = nh.advertise<sgtdv_msgs::ConeArr>("slam/map", 1);
  
  ros::spin();

  return 0;
}