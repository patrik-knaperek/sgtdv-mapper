#pragma once

#include <ros/ros.h>
#include <fsd_common_msgs/CarState.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/Point2D.h>
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

        void carStateCallbackSim(const fsd_common_msgs::CarState::ConstPtr& msg);
        void carStateCallbackReal(const sgtdv_msgs::CarPose::ConstPtr& msg);
        void conesCallbackSim(const sensor_msgs::PointCloud2::ConstPtr& msg);        
        void conesCallbackReal(const sgtdv_msgs::ConeArr::ConstPtr& msg);
        void dataAssEuclid();
        void pubCarState();
        void pubCones();

        float euclidThresh;

    private:

        double m_odomX, m_odomY, m_odomTheta, m_coneRange, m_coneBearing, m_coneAbsX, m_coneAbsY, m_euclidDist;
        double m_coneColor;
        
        vector<vector<double> > m_coneAbsVect;
        vector<vector<double> > m_coneMap;

        geometry_msgs::Point pointCarPose; 
        geometry_msgs::Point pointCone;
        std_msgs::ColorRGBA coneRGBA;
        sgtdv_msgs::CarPose carPose;
        sgtdv_msgs::ConeArr coneArr;         
        sgtdv_msgs::Cone cone; 
        visualization_msgs::Marker carPoseMarker;
        visualization_msgs::Marker coneMarker;
        
};