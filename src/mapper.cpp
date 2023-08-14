
#include "../include/mapper/mapper.h"

using namespace std;

Mapper::Mapper()
{
}

  void Mapper::carPoseCallback(const sgtdv_msgs::CarPose::ConstPtr& msg)
{  
  m_carPose.position.x = msg->position.x;
  m_carPose.position.y = msg->position.y;
  m_carPose.yaw = msg->yaw;

  pubCarPose.publish(m_carPose);
}

void Mapper::conesCallback(const sgtdv_msgs::ConeStampedArr::ConstPtr& msg)
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
  pubMap.publish(coneArr);
}

int main(int argc, char **argv)
{
  Mapper mapper; 

  ros::init(argc, argv, "mapper");
  ros::NodeHandle nh;
  
  nh.getParam("euclid_threshold", mapper.euclidThresh);
  ros::Subscriber carStateSubReal = nh.subscribe("pose_estimate", 1, &Mapper::carPoseCallback, &mapper);
  ros::Subscriber conesSubReal = nh.subscribe("fusion_cones", 1, &Mapper::conesCallback, &mapper);
  mapper.pubCarPose = nh.advertise<sgtdv_msgs::CarPose>("slam/pose", 1);
  mapper.pubMap = nh.advertise<sgtdv_msgs::ConeArr>("slam/map", 1);
  
  ros::spin();

  return 0;
}