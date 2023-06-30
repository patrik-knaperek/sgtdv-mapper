
#include "../include/mapper/mapper.h"

using namespace std;

Mapper::Mapper()
{
}

void Mapper::carStateCallbackSim(const fsd_common_msgs::CarState::ConstPtr& msg)
{  
  m_carPose.position.x = msg->car_state.x;
  m_carPose.position.y = msg->car_state.y;
  m_carPose.yaw = msg->car_state.theta;

  pubCarPose.publish(m_carPose);
  visCarState();
}

  void Mapper::carStateCallbackReal(const sgtdv_msgs::CarPose::ConstPtr& msg)
{  
  m_carPose.position.x = msg->position.x;
  m_carPose.position.y = msg->position.y;
  m_carPose.yaw = msg->yaw;

  pubCarPose.publish(m_carPose);
  visCarState();
}

void Mapper::conesCallbackSim(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  m_coneAbsVect.clear();
  float const *temp;

  for (int i = 0; i < msg->width; i++)
    {

      temp = reinterpret_cast<const float*>(&msg->data[i*msg->point_step]);
      geometry_msgs::Point32 point;    

      point.x = *temp;
      point.y = *(temp + 1);

      if(*(temp + 9) > 0.85){m_coneColor = 1;}   // 1 = blue
      if(*(temp + 10) > 0.85){m_coneColor = 2;}   // 2 = yellow
      if(*(temp + 11) > 0.85){m_coneColor = 3;}   // 3 = orange

      m_coneRange = sqrt(pow(point.x,2) + pow(point.y,2));
      m_coneBearing = atan2(point.y, point.x);

      m_coneAbsX = m_carPose.position.x + m_coneRange * cos(m_coneBearing + m_carPose.yaw);
      m_coneAbsY = m_carPose.position.y + m_coneRange * sin(m_coneBearing + m_carPose.yaw);

      dataAssEuclid();
    }
  
  pubCones();
} 

void Mapper::conesCallbackReal(const sgtdv_msgs::ConeStampedArr::ConstPtr& msg)
{
  m_coneAbsVect.clear();
  geometry_msgs::Point32 point;

  for (int i = 0; i < msg->cones.size(); i++)
    {
      if(msg->cones[i].color == 'b'){m_coneColor = 1;}   // 'b' = blue
      if(msg->cones[i].color == 'y'){m_coneColor = 2;}   // 'y' = yellow
      if(msg->cones[i].color == 's' || msg->cones[i].color == 'g'){m_coneColor = 3;}   // 's' = orange small; 'g' = orange big

      point.x = msg->cones[i].coords.x;
      point.y = msg->cones[i].coords.y;
      m_coneRange = sqrt(pow(point.x,2) + pow(point.y,2));
      m_coneBearing = atan2(point.y, point.x);

      m_coneAbsX = m_carPose.position.x + m_coneRange * cos(m_coneBearing + m_carPose.yaw);
      m_coneAbsY = m_carPose.position.y + m_coneRange * sin(m_coneBearing + m_carPose.yaw);

      dataAssEuclid();
    }
  
  pubCones();
}

void Mapper::dataAssEuclid(){
  
  vector<double> newRow;
  vector<double> euclidVect;  
  euclidVect.clear();
 
  if(m_coneMap.empty() == true){
    newRow = {m_coneAbsX, m_coneAbsY, m_coneColor, 1};
    m_coneMap.push_back(newRow);
  } 
  else{
   
    for(int i=0; i < m_coneMap.size(); i++)
    {
      m_euclidDist = sqrt(pow(m_coneAbsX - m_coneMap[i][0], 2) + pow(m_coneAbsY - m_coneMap[i][1], 2));
      euclidVect.push_back(m_euclidDist);
    }

    int minElementIndex = std::min_element(euclidVect.begin(),euclidVect.end()) - euclidVect.begin();

    if(euclidVect[minElementIndex] < euclidThresh){
      m_coneMap[minElementIndex][0] = m_coneAbsX;
      m_coneMap[minElementIndex][1] = m_coneAbsY;
      
      /* color decision */
      if (m_coneMap[minElementIndex][2] == m_coneColor)
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
          m_coneMap[minElementIndex][2] = m_coneColor;
          m_coneMap[minElementIndex][3] = 1;
        }
      }
    }
    else{
      newRow = {m_coneAbsX, m_coneAbsY, m_coneColor, 1};
      m_coneMap.push_back(newRow); 
    }
  }
}

void Mapper::visCarState(){

  static visualization_msgs::Marker carPoseMarker;
  geometry_msgs::Point pointCarPose;
  static int count = 0;
  if (!(count++ % 100)) 
  {
    carPoseMarker.header.frame_id =  "map";
    carPoseMarker.header.stamp = ros::Time();
    carPoseMarker.type = visualization_msgs::Marker::POINTS;
    carPoseMarker.action = visualization_msgs::Marker::ADD;
    carPoseMarker.id = 0;
    carPoseMarker.lifetime = ros::Duration(0);
    carPoseMarker.scale.x = 0.3;
    carPoseMarker.scale.y = 0.3;
    carPoseMarker.color.a = 1;
    carPoseMarker.color.r = 1;

    pointCarPose.x = m_carPose.position.x;
    pointCarPose.y = m_carPose.position.y;

    carPoseMarker.points.push_back(pointCarPose);
    pubCarPoseMarker.publish(carPoseMarker);
  }
}

void Mapper::pubCones(){

  vector<vector<double>>::iterator iter;
  static sgtdv_msgs::ConeArr coneArr;
  coneArr.cones.clear();

  static visualization_msgs::Marker coneMarker;
  coneMarker.points.clear();
  coneMarker.colors.clear();
  coneMarker.action = visualization_msgs::Marker::DELETEALL;

  coneMarker.header.frame_id = "map";
  coneMarker.header.stamp = ros::Time();
  coneMarker.type = visualization_msgs::Marker::POINTS;
  coneMarker.action = visualization_msgs::Marker::ADD;
  
  sgtdv_msgs::Cone cone;
  geometry_msgs::Point pointCone;
  std_msgs::ColorRGBA coneRGBA;
  for(iter = m_coneMap.begin(); iter < m_coneMap.end(); ++iter)
  {
    cone.coords.x = m_coneMap[iter-m_coneMap.begin()][0];
    cone.coords.y = m_coneMap[iter-m_coneMap.begin()][1];
    cone.color = m_coneMap[iter-m_coneMap.begin()][2];
    coneArr.cones.push_back(cone);
    
    coneMarker.scale.x = 0.2;
    coneMarker.scale.y = 0.2;
      
    pointCone.x = m_coneMap[iter-m_coneMap.begin()][0];
    pointCone.y = m_coneMap[iter-m_coneMap.begin()][1];
    
    if(cone.color == 1){
      coneRGBA.r = 0;     // blue
      coneRGBA.g = 0;
      coneRGBA.b = 1;
    }
    else if(cone.color == 2) {
      coneRGBA.r = 1;     //yellow
      coneRGBA.g = 1;
      coneRGBA.b = 0;
    }
    else if(cone.color == 3) {
      coneRGBA.r = 1;     // orange
      coneRGBA.g = 0.55;
      coneRGBA.b = 0;   
    }
    coneRGBA.a = 1;
    
    coneMarker.points.push_back(pointCone);
    coneMarker.colors.push_back(coneRGBA);
    }
    pubMapMarker.publish(coneMarker); 

  pubMap.publish(coneArr);

}

int main(int argc, char **argv)
{
  Mapper mapper; 

  ros::init(argc, argv, "mapper");
  ros::NodeHandle nh;
  
  nh.getParam("euclid_threshold", mapper.euclidThresh);
  ros::Subscriber carStateSubSim = nh.subscribe("estimation/slam/state", 1, &Mapper::carStateCallbackSim, &mapper);
  // ros::Subscriber conesSubSim = nh.subscribe("fssim/camera/cones", 1, &Mapper::conesCallbackSim, &mapper);
  ros::Subscriber carStateSubReal = nh.subscribe("car_state", 1, &Mapper::carStateCallbackReal, &mapper);
  ros::Subscriber conesSubReal = nh.subscribe("fusion_cones", 1, &Mapper::conesCallbackReal, &mapper);
  mapper.pubCarPose = nh.advertise<sgtdv_msgs::CarPose>("slam/pose", 1);
  mapper.pubMap = nh.advertise<sgtdv_msgs::ConeArr>("slam/map", 1);
  mapper.pubCarPoseMarker = nh.advertise<visualization_msgs::Marker>("slam/pose/marker", 1);
  mapper.pubMapMarker = nh.advertise<visualization_msgs::Marker>("slam/map/marker", 1);

  ros::spin();

  return 0;
}