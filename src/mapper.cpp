
#include "../include/mapper/mapper.h"

using namespace std;

Mapper::Mapper(){
  
  m_odomX = 0.f;
  m_odomY = 0.f; 
  m_odomTheta = 0.f;
}

void Mapper::carStateCallbackSim(const fsd_common_msgs::CarState::ConstPtr& msg)
{  
  m_odomX = msg->car_state.x;
  m_odomY = msg->car_state.y;
  m_odomTheta = msg->car_state.theta;

  pubCarState();
}

  void Mapper::carStateCallbackReal(const sgtdv_msgs::CarPose::ConstPtr& msg)
{  
  m_odomX = msg->position.x;
  m_odomY = msg->position.y;
  m_odomTheta = msg->yaw;

  pubCarState();
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

      m_coneAbsX = m_odomX + m_coneRange * cos(m_coneBearing + m_odomTheta);
      m_coneAbsY = m_odomY + m_coneRange * sin(m_coneBearing + m_odomTheta);

      dataAssEuclid();

      vector<double> newRow;
      newRow = {m_coneAbsX, m_coneAbsY, m_coneColor};

      m_coneAbsVect.push_back(newRow);
    }
  
  pubCones();
} 

void Mapper::conesCallbackReal(const sgtdv_msgs::ConeArr::ConstPtr& msg)
{
  m_coneAbsVect.clear();
  float const *temp;
  geometry_msgs::Point32 point;

  for (int i = 0; i < msg->cones.size(); i++)
    {
      if(*(temp + 9) > 0.85){m_coneColor = 1;}   // 1 = blue
      if(*(temp + 10) > 0.85){m_coneColor = 2;}   // 2 = yellow
      if(*(temp + 11) > 0.85){m_coneColor = 3;}   // 3 = orange

      m_coneRange = sqrt(pow(point.x,2) + pow(point.y,2));
      m_coneBearing = atan2(point.y, point.x);

      m_coneAbsX = m_odomX + m_coneRange * cos(m_coneBearing + m_odomTheta);
      m_coneAbsY = m_odomY + m_coneRange * sin(m_coneBearing + m_odomTheta);

      dataAssEuclid();

      vector<double> newRow;
      newRow = {m_coneAbsX, m_coneAbsY, cone.color};

      m_coneAbsVect.push_back(newRow);
    }
  
  pubCones();
}

void Mapper::dataAssEuclid(){
  
  vector<double> newRow1, newRow2;
  vector<double> euclidVect;  
  euclidVect.clear();
 
  if(m_coneMap.empty() == true){
    newRow1 = {m_coneAbsX, m_coneAbsY, m_coneColor};
    m_coneMap.push_back(newRow1);
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
    }
    else{
      newRow2 = {m_coneAbsX, m_coneAbsY, m_coneColor};
      m_coneMap.push_back(newRow2); 
    }
  }
}

void Mapper::pubCarState(){

  carPose.position.x = m_odomX;
  carPose.position.y = m_odomY;
  pubCarPose.publish(carPose);

  carPoseMarker.header.frame_id =  "/map";
  carPoseMarker.header.stamp = ros::Time();
  carPoseMarker.type = visualization_msgs::Marker::POINTS;
  carPoseMarker.action = visualization_msgs::Marker::ADD;
  carPoseMarker.id = 0;
  carPoseMarker.lifetime = ros::Duration(0);
  carPoseMarker.scale.x = 0.3;
  carPoseMarker.scale.y = 0.3;
  carPoseMarker.color.a = 1;
  carPoseMarker.color.r = 1;

  pointCarPose.x = m_odomX;
  pointCarPose.y = m_odomY;

  carPoseMarker.points.push_back(pointCarPose);
  pubCarPoseMarker.publish(carPoseMarker);
}

void Mapper::pubCones(){

  vector<vector<double>>::iterator iter;
  int i = 0;

  for(iter = m_coneMap.begin(); iter < m_coneMap.end(); ++iter)
  {

    coneMarker.points.clear();
    coneMarker.colors.clear();
    coneMarker.action = visualization_msgs::Marker::DELETEALL;

    cone.coords.x = m_coneMap[iter-m_coneMap.begin()][0];
    cone.coords.y = m_coneMap[iter-m_coneMap.begin()][1];
    cone.color = m_coneMap[iter-m_coneMap.begin()][2];
    coneArr.cones.push_back(cone);
    

    coneMarker.header.frame_id = "map";
    coneMarker.header.stamp = ros::Time();
    coneMarker.type = visualization_msgs::Marker::POINTS;
    coneMarker.action = visualization_msgs::Marker::ADD;

    coneMarker.id = i++;
    coneMarker.scale.x = 0.2;
    coneMarker.scale.y = 0.2;
      

    pointCone.x = m_coneMap[iter-m_coneMap.begin()][0];
    pointCone.y = m_coneMap[iter-m_coneMap.begin()][1];
    coneRGBA.a = 1;

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
    
    coneMarker.points.push_back(pointCone);
    coneMarker.colors.push_back(coneRGBA);
    pubMapMarker.publish(coneMarker);  
    }

  pubMap.publish(coneArr);

}

int main(int argc, char **argv)
{
  Mapper mapper; 

  ros::init(argc, argv, "mapper");
  ros::NodeHandle nh;
  
  nh.getParam("euclid_threshold", mapper.euclidThresh);
  ros::Subscriber carStateSubSim = nh.subscribe("estimation/slam/state", 1, &Mapper::carStateCallbackSim, &mapper);
  ros::Subscriber conesSubSim = nh.subscribe("fssim/camera/cones", 1, &Mapper::conesCallbackSim, &mapper);
  ros::Subscriber carStateSubReal = nh.subscribe("car_state", 1, &Mapper::carStateCallbackReal, &mapper);
  ros::Subscriber conesSubReal = nh.subscribe("fusion_cones", 1, &Mapper::conesCallbackReal, &mapper);
  mapper.pubCarPose = nh.advertise<sgtdv_msgs::CarPose>("slam/pose", 1);
  mapper.pubMap = nh.advertise<sgtdv_msgs::ConeArr>("slam/map", 1);
  mapper.pubCarPoseMarker = nh.advertise<visualization_msgs::Marker>("slam/pose/marker", 1);
  mapper.pubMapMarker = nh.advertise<visualization_msgs::Marker>("slam/map/marker", 1);

  ros::spin();

  return 0;
}