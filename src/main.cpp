#include <ros/ros.h>
#include "color_detection_cpp/camera_subscriber.hpp"
#include "color_detection_cpp/velocity_publisher.hpp"

int main(int argc, char** argv){
    // Init ROS node
    ros::init(argc, argv, "color_detection_cpp_node");
    ros::NodeHandle nh("~"); 

    // Define parameters 
    std::string topic_camara, topic_velocidad;
    int min_detection_area;
    double linear = 0.0, angular = 0.0;
    nh.param<int>("min_detection_area", min_detection_area, 30000);
    nh.param<std::string>("camera_topic", topic_camara, "/camera/rgb/image_raw");
    nh.param<std::string>("velocity_topic", topic_velocidad, "/cmd_vel");

    // Initialize camera subscriber and velocity publisher
    CameraSubscriber camera(nh, topic_camara, min_detection_area);
    VelocityPublisher velocity(nh, topic_velocidad);

    // Main loop
    ros::Rate rate(5); 
    while (ros::ok()){

        // Get last detected object data 
        CameraSubscriber::DetectionData dd = camera.getLastDetection();
        
        // Define velocity
        if(dd.area>min_detection_area){
            velocity.calculateVelocity(linear, angular, dd);

        } else{
            linear = 0.0;
            angular = 0.6;
        }
        
        // Publish velocity
        velocity.publish(linear, angular);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
