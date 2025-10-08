#include "color_detection_cpp/camera_subscriber.hpp"


CameraSubscriber::CameraSubscriber(ros::NodeHandle& nh, const std::string& topic, int min_detection_area){
    // Constructor: init subscriber, parameters and color range 
    sub_ = nh.subscribe(topic, 1, &CameraSubscriber::callback, this);
    min_detection_area_ = min_detection_area;
    loadParameters(nh);
    setColorRange();
}

void CameraSubscriber::loadParameters(ros::NodeHandle &nh){   
    // Load some parameters for color detection 
    nh.param<std::string>("target_color", target_color_, "red");
    nh.param<double>("image_scale_factor", image_scale_factor_, 0.3);
    ROS_INFO("Camera params loaded: target_color=%s, min_detection_area=%d, image_scale_factor=%.2f",
             target_color_.c_str(), min_detection_area_, image_scale_factor_);
}

void CameraSubscriber::setColorRange(){

    // Define color ranges for detection depending on target_color
    if (target_color_ == "red") {
        lower_range_ = cv::Scalar(0, 200, 102);
        upper_range_ = cv::Scalar(15, 255, 255);
    } 
    else if (target_color_ == "green") {
        lower_range_ = cv::Scalar(40, 50, 50);
        upper_range_ = cv::Scalar(80, 255, 255);
    } 
    else if (target_color_ == "blue") {
        lower_range_ = cv::Scalar(100, 150, 50);
        upper_range_ = cv::Scalar(140, 255, 255);
    } 
    else if (target_color_ == "yellow") {
        lower_range_ = cv::Scalar(20, 100, 100);
        upper_range_ = cv::Scalar(30, 255, 255);
    } 
    else {
        ROS_ERROR("Unsupported target_color: %s", target_color_.c_str());
        throw std::invalid_argument("Unsupported color: " + target_color_);
    }

    ROS_INFO("HSV range for '%s': lower=(%.0f, %.0f, %.0f), upper=(%.0f, %.0f, %.0f)",
             target_color_.c_str(),
             lower_range_[0], lower_range_[1], lower_range_[2],
             upper_range_[0], upper_range_[1], upper_range_[2]);
}

void CameraSubscriber::callback(const sensor_msgs::ImageConstPtr& msg){
    try{
        // Convert ROS image to OpenCV
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        ROS_INFO_STREAM("Image received, size: " << frame.cols << "x" << frame.rows);
        processImage(frame); // Process image for color detection
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Eroor converting image: %s", e.what());
    }
}

cv::Mat CameraSubscriber::detectColor(const cv::Mat &image){
        cv::Mat blurred_image;
        cv::Mat hsv_image;
        cv::Mat mask;
        cv::Mat color_mask;

        // Apply gaussian filter and detect color using HSV
        cv::GaussianBlur(image, blurred_image, cv::Size(15, 15), 0);
        cv::cvtColor(blurred_image, hsv_image, cv::COLOR_BGR2HSV);
        cv::inRange(hsv_image, lower_range_, upper_range_, mask);
        
        // Morphological opening and closing to remove noise
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25));
        cv::morphologyEx(mask, color_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(color_mask, color_mask, cv::MORPH_CLOSE, kernel);

        return color_mask;
}

CameraSubscriber::DetectionData CameraSubscriber::getMaxContour(const cv::Mat& binary_mask, double min_area){
    
    // Get max contour detected of target color in image and extract the contour area and center
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(binary_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    CameraSubscriber::DetectionData dd;
    dd.area = min_area;
    dd.center = cv::Point(-1, -1);

    for (const auto& contour : contours) {
        double current_area = cv::contourArea(contour);
        if (current_area > dd.area) {
            dd.area = current_area;
            dd.contour = contour;

            cv::Moments M = cv::moments(contour);
            if (M.m00 != 0) {
                dd.center = cv::Point(static_cast<int>(M.m10 / M.m00),
                                          static_cast<int>(M.m01 / M.m00));
            }
        }
    }
    return dd;
}

void CameraSubscriber::processImage(const cv::Mat &image){
    
    // Get image dimensions for navigation calculations
    int image_height = image.rows;
    int image_width  = image.cols;
    int image_center_x = image_width / 2;

    // Detect color and extract maximum detection (max contour)
    cv::Mat color_mask = detectColor(image);
    CameraSubscriber::DetectionData dd = getMaxContour(color_mask);
    dd.image_center_x = image_center_x;
    last_detection_ = dd;

    // Visualize image and detections
    CameraSubscriber::visualize(image.clone(), color_mask, dd);
    if (dd.area > min_detection_area_){
        ROS_INFO("Target color detected - area: %.2f, center: (%d,%d)", dd.area, dd.center.x, dd.center.y);        
    }
    else{
        ROS_WARN("No significant color detection - initiating search");
    }
}


CameraSubscriber::DetectionData CameraSubscriber::getLastDetection(){
    return last_detection_;
}

void CameraSubscriber::visualize(cv::Mat image, cv::Mat &color_mask, const DetectionData &dd){
    // Visualize image and detection 
    if (dd.area > min_detection_area_){

        std::vector<std::vector<cv::Point>> contours_to_draw = { dd.contour };
        cv::drawContours(image, contours_to_draw, -1, cv::Scalar(0, 255, 0), 2);
        
        cv::circle(image, dd.center, 8, cv::Scalar(0, 200, 200), -1);
        cv::circle(image, dd.center, 12, cv::Scalar(0, 200, 200), 2);
        
        std::string target_text = "Target: " + target_color_;
        std::string position_text = "Center: (" + std::to_string(dd.center.x) + ", " + std::to_string(dd.center.y) + ")";
        std::string area_text = "Area: " + std::to_string(static_cast<int>(dd.area));

        cv::putText(image, target_text, cv::Point(15, 50),
                    cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(255, 255, 255), 2);
        cv::putText(image, position_text, cv::Point(15, 100),
                    cv::FONT_HERSHEY_SIMPLEX, 1.4, cv::Scalar(255, 255, 255), 2);
        cv::putText(image, area_text, cv::Point(15, 150),
                    cv::FONT_HERSHEY_SIMPLEX, 1.4, cv::Scalar(255, 255, 255), 2);
    }

    cv::Mat resized_image, resized_color_mask;
    float fx=0.3, fy=0.3;
    cv::resize(image, resized_image, cv::Size(), fx, fy, cv::INTER_AREA);
    cv::resize(color_mask, resized_color_mask, cv::Size(), fx, fy, cv::INTER_AREA);

    cv::imshow("Camera", resized_image);
    cv::imshow("Color Mask", resized_color_mask);
    cv::waitKey(1);
}