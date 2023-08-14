#pragma once

#include <ros/ros.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/ConeStampedArr.h>
#include <sgtdv_msgs/Point2DStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointField.h>

#include <cmath>
#include <vector>

using namespace std;

class Mapper{
    
    public:
        Mapper();
        
        ros::Publisher pubMap;
        ros::Publisher pubCarPose;
        ros::Publisher pubMapMarker;
        ros::Publisher pubCarPoseMarker;

        void carPoseCallback(const sgtdv_msgs::CarPose::ConstPtr& msg);
        void conesCallback(const sgtdv_msgs::ConeStampedArr::ConstPtr& msg);
        void dataAssEuclid();
        void pubCones();

        float euclidThresh;

    private:

        double m_coneRange, m_coneBearing, m_coneAbsX, m_coneAbsY, m_euclidDist;
        double m_coneColor;
        
        vector<vector<double> > m_coneAbsVect;
        vector<vector<double> > m_coneMap;

        sgtdv_msgs::CarPose m_carPose;
};