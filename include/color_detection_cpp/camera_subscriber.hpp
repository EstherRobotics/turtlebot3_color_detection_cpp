#ifndef CAMERA_SUBSCRIBER_HPP
#define CAMERA_SUBSCRIBER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraSubscriber{
    public:

        // Struct to manage color detection data 
        struct DetectionData {
            std::vector<cv::Point> contour;
            double area;
            cv::Point center;
            int image_center_x;
        };

        CameraSubscriber(ros::NodeHandle &nh, const std::string &topic, int min_detection_area);
        void callback(const sensor_msgs::ImageConstPtr &msg);
        void loadParameters(ros::NodeHandle &nh);
        void setColorRange();
        cv::Mat detectColor(const cv::Mat &image);
        void processImage(const cv::Mat &image);
        void visualize(cv::Mat image, cv::Mat &color_mask, const DetectionData &dd);
        DetectionData getMaxContour(const cv::Mat& binary_mask, double min_area = 1000.0);
        DetectionData getLastDetection();


    private:
        ros::Subscriber sub_;
        std::string target_color_;
        int min_detection_area_;
        double image_scale_factor_;
        cv::Scalar lower_range_;
        cv::Scalar upper_range_;
        DetectionData last_detection_;
};

#endif
