/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Martin Lučan, Patrik Knaperek, Filip Botka
/*****************************************************/

#pragma once

/* ROS */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

/* SGT */
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/ConeWithCovStampedArr.h>
#include "SGT_Macros.h"

class Mapper
{    
  public:
    explicit Mapper(ros::NodeHandle& nh);
    ~Mapper() = default;
    
  private:
    void carPoseCallback(const sgtdv_msgs::CarPose::ConstPtr& msg) const;
    void conesCallback(const sgtdv_msgs::ConeWithCovStampedArr::ConstPtr& msg);
    void conesCallbackSim(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void dataAssEuclid(const double new_x, const double new_y, const double new_color);
    void pubCones() const;

    struct Params
    {
      float euclid_th_;
    } params_;

    ros::Publisher pub_map_;
    ros::Publisher pub_car_pose_;
    ros::Subscriber car_pose_sub_;
    ros::Subscriber cones_sub_;

    std::vector<std::vector<double> > cone_map_;

    sgtdv_msgs::CarPose car_pose_;
    tf::TransformListener listener_;

  #ifdef SGT_DEBUG_STATE
    ros::Publisher vis_debug_pub_;
  #endif
};