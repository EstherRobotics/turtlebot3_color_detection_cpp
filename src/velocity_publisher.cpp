#include "color_detection_cpp/velocity_publisher.hpp"

VelocityPublisher::VelocityPublisher(ros::NodeHandle &nh, const std::string &topic){
    // Constructor: advertise ROS topic for velocity publisher, load parameters
    pub_ = nh.advertise<geometry_msgs::Twist>(topic, 10);
    loadParameters(nh);
}

void VelocityPublisher::loadParameters(ros::NodeHandle &nh){
    ros::NodeHandle pnh("~");  // Private node handle for parameters
    pnh.param<int>("max_area_threshold", max_area_threshold_, 450000);
    pnh.param<int>("min_area_threshold", min_area_threshold_, 300000);
    pnh.param<double>("max_linear_speed", max_linear_speed_, 0.15);
    pnh.param<double>("max_angular_speed", max_angular_speed_, 0.15);
    pnh.param<int>("center_limit_x", center_limit_x_, 100);

    ROS_INFO("Velocity params loaded: max_area=%d, min_area=%d, max_linear=%.2f, max_angular=%.2f, offset=%d",
             max_area_threshold_, min_area_threshold_,
             max_linear_speed_, max_angular_speed_, center_limit_x_);
}

void VelocityPublisher::calculateVelocity(double &linear, double &angular, const CameraSubscriber::DetectionData &dd){
    // Calculates linear and angular velocities
    calculateLinearVelocity(linear, dd);
    calculateAngularVelocity(linear, angular, dd);
}    

void VelocityPublisher::calculateLinearVelocity(double &linear, const CameraSubscriber::DetectionData &dd){
    // Calculates linear velocity
    // Object too close - move backward
    if (dd.area > max_area_threshold_){
        ROS_INFO("Object too close - moving backward");
        linear = -0.1;
    }
    // Object too far - move forward
    else if (dd.area < min_area_threshold_){
        ROS_INFO("Object detected - approaching");
        linear = max_linear_speed_;
    }
    // Object at optimal distance - maintain position
    else{
        ROS_INFO("Object at optimal distance - maintaining position");
        linear = 0.0;
    }
}

void VelocityPublisher::calculateAngularVelocity(double &linear, double &angular, const CameraSubscriber::DetectionData &dd){
    // Calculates angular velocity 
    angular = 0.0;
    if (linear > 0.0){
        // Calculate x distance from center
        int distance_x = (dd.center.x - dd.image_center_x);
        
        // No turning needed if object is centered
        if (std::abs(distance_x) >= center_limit_x_){
            // Calculate proportional angular velocity depending on distance
            double proportional_gain = 0.002;
            angular =-proportional_gain * distance_x;
            
            // Limit angular velocity to maximum allowed
            angular = std::max(std::min(angular, max_angular_speed_), -max_angular_speed_);
        }
    
        // Print turning direction
        if (angular>0.0){
            ROS_INFO("Turning left to center object (distance (x): %.2dpx)", distance_x);
        }
        else if (angular<0.0){
            ROS_INFO("Turning right to center object (distance (x): %.2dpx)", distance_x);
        }
    }
}

void VelocityPublisher::publish(double linear, double angular){
    // Publish velocity
    geometry_msgs::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;
    pub_.publish(msg);
    ROS_INFO("Velocity published, x: %.2f, z: %.2f", linear, angular);

}

