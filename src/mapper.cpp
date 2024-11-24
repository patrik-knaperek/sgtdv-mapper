
/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Martin Lučan, Patrik Knaperek, Filip Botka
/*****************************************************/

/* ROS */
#include <geometry_msgs/Point.h>

/* SGT */
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/Point2DStamped.h>
#include <sgtdv_msgs/DebugState.h>
#include "SGT_Utils.h"

/* Header */
#include "mapper.h"

Mapper::Mapper(ros::NodeHandle& nh) :
  /* ROS interface initialization */
  pub_car_pose_(nh.advertise<sgtdv_msgs::CarPose>("slam/pose", 1)),
  pub_map_(nh.advertise<sgtdv_msgs::ConeArr>("slam/map", 1)),

  car_pose_sub_(nh.subscribe("odometry/pose", 1, &Mapper::carPoseCallback, this)),
  cones_sub_(nh.subscribe("fusion/cones", 1, &Mapper::conesCallback, this))
  //, cones_sub_(nh.subscribe("fssim/camera/cones", 1, &Mapper::conesCallbackSim, this)
#ifdef SGT_DEBUG_STATE
  , vis_debug_pub_(nh.advertise<sgtdv_msgs::DebugState>("slam/debug_state", 2))
#endif
{
  /* Load parameters */
  Utils::loadParam(nh, "euclid_threshold", 0.3f, &params_.euclid_th_);
}

void Mapper::carPoseCallback(const sgtdv_msgs::CarPose::ConstPtr& msg) const
{  
  sgtdv_msgs::CarPose carPose;
  carPose.position.x = msg->position.x;
  carPose.position.y = msg->position.y;
  carPose.yaw = msg->yaw;

  pub_car_pose_.publish(carPose);
}

void Mapper::conesCallback(const sgtdv_msgs::ConeWithCovStampedArr::ConstPtr& msg)
{
#ifdef SGT_DEBUG_STATE
  sgtdv_msgs::DebugState state;
  state.stamp = ros::Time::now();
  state.working_state = 1;
  vis_debug_pub_.publish(state);
#endif /* SGT_DEBUG_STATE */
  
  geometry_msgs::PointStamped coords_base, coords_map;
  double new_color;

  for(const auto& cone : msg->cones)
  {
    new_color = cone.color;

    coords_base.header = cone.coords.header;
    coords_base.point.x = cone.coords.x;
    coords_base.point.y = cone.coords.y;

    try
    {
      listener_.transformPoint("map", coords_base, coords_map);
    }
    catch(tf::TransformException &e)
    {
      ROS_WARN_STREAM(e.what());
      continue;
    }

    dataAssEuclid(coords_map.point.x, coords_map.point.y, new_color);
  }
  
  pubCones();

#ifdef SGT_DEBUG_STATE
  state.stamp = ros::Time::now();
  state.working_state = 0;
  state.num_of_cones = cone_map_.size();
  vis_debug_pub_.publish(state);
#endif /* SGT_DEBUG_STATE */
}

void Mapper::conesCallbackSim(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  float const *temp;
  double new_color;

  for(int i = 0; i < msg->width; i++)
  {

    temp = reinterpret_cast<const float*>(&msg->data[i*msg->point_step]);
    geometry_msgs::PointStamped coords_base, coords_map;

    coords_base.header = msg->header;
    coords_base.point.x = *temp;
    coords_base.point.y = *(temp + 1);
    
    if(*(temp + 9) > 0.85){new_color = 'b';} // blue
    if(*(temp + 10) > 0.85){new_color = 'y';} // yellow
    if(*(temp + 11) > 0.85){new_color = 's';} // orange small

    try
    {
      listener_.transformPoint("map", coords_base, coords_map);
    }
    catch(tf::TransformException &e)
    {
      ROS_WARN_STREAM(e.what());
      continue;
    }

    dataAssEuclid(coords_map.point.x, coords_map.point.y, new_color);
  }
  
  pubCones();
} 

void Mapper::dataAssEuclid(const double new_x, const double new_y, const double new_color)
{
  
  std::vector<double> new_row;
  std::vector<double> euclid_vect;  
 
  if(cone_map_.empty() == true){
    new_row = {new_x, new_y, new_color, 1};
    cone_map_.push_back(new_row);
  } 
  else
  {  
    for(const auto& cone : cone_map_)
    {
      const double euclid_dist = sqrt(pow(new_x - cone[0], 2) + pow(new_y - cone[1], 2));
      euclid_vect.emplace_back(euclid_dist);
    }

    const int min_element_idx = std::min_element(euclid_vect.begin(), euclid_vect.end()) - euclid_vect.begin();

    if(euclid_vect[min_element_idx] < params_.euclid_th_)
    {
      cone_map_[min_element_idx][0] = new_x;
      cone_map_[min_element_idx][1] = new_y;
      
      /* color decision */
      if(cone_map_[min_element_idx][2] == new_color)
      {
        cone_map_[min_element_idx][3]++;
      }
      else
      {
        if(cone_map_[min_element_idx][3] > 0)
        {
          cone_map_[min_element_idx][3] -= 5;
        }
        else
        {
          cone_map_[min_element_idx][2] = new_color;
          cone_map_[min_element_idx][3] = 1;
        }
      }
    }
    else
    {
      new_row = {new_x, new_y, new_color, 1};
      cone_map_.push_back(new_row); 
    }
  }
}

void Mapper::pubCones() const
{
  static sgtdv_msgs::ConeArr cone_arr;
  cone_arr.cones.clear();

  sgtdv_msgs::Cone cone;
  for(const auto& iter : cone_map_)
  {
    cone.coords.x = iter[0];
    cone.coords.y = iter[1];
    cone.color = iter[2];
    cone_arr.cones.push_back(cone);
  }
  pub_map_.publish(cone_arr);
}