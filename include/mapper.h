/*****************************************************/
//Organization: Stuba Green Team
//Authors: Martin Lučan, Patrik Knaperek, Filip Botka
/*****************************************************/

#pragma once

/* C++ */
#include <cmath>
#include <vector>

/* ROS */
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

/* SGT */
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/ConeStampedArr.h>
#include <sgtdv_msgs/Point2DStamped.h>
#include "../../SGT_Utils.h"

class Mapper{
    
	public:
		Mapper(ros::NodeHandle& nh);
		~Mapper() = default;
		
		void carPoseCallback(const sgtdv_msgs::CarPose::ConstPtr& msg);
		void conesCallback(const sgtdv_msgs::ConeStampedArr::ConstPtr& msg);
		void conesCallbackSim(const sensor_msgs::PointCloud2::ConstPtr& msg);
		void dataAssEuclid(const double new_x, const double new_y, const double new_color);
		void pubCones();

		struct Params
		{
			float euclid_th_;
		};

	private:
		ros::Publisher pub_map_;
		ros::Publisher pub_car_pose_;
		ros::Subscriber car_pose_sub_;
		ros::Subscriber cones_sub_;

		Params params_;

		std::vector<std::vector<double> > cone_map_;

		sgtdv_msgs::CarPose m_carPose;
		tf::TransformListener listener_;
};