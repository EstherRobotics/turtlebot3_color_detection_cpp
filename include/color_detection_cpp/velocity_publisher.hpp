
#ifndef VELOCITY_PUBLISHER_HPP
#define VELOCITY_PUBLISHER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "color_detection_cpp/camera_subscriber.hpp"

class VelocityPublisher{
    public:
        VelocityPublisher(ros::NodeHandle &nh, const std::string &topic);
        void loadParameters(ros::NodeHandle &nh);
        void calculateVelocity(double &linear, double &angular, const CameraSubscriber::DetectionData &dd);
        void calculateLinearVelocity(double &linear, const CameraSubscriber::DetectionData &dd);
        void calculateAngularVelocity(double &linear, double &angular, const CameraSubscriber::DetectionData &dd);
        void publish(double linear, double angular);

    private: 
        ros::Publisher pub_;
        int max_area_threshold_;
        int min_area_threshold_;
        double max_linear_speed_;
        double max_angular_speed_;
        int center_limit_x_;
};

#endif