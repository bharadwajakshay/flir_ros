#ifndef FLIR_ROS_H_
#define FLIR_ROS_H_
#endif

#include <iostream>
#include <cstdio>
#include <memory>

#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <thread>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <opencv2/opencv.hpp>

class flirROS2 : public rclcpp::Node
{

    public:
    flirROS2();

    private:
    /*************** Variables ********************/
    Spinnaker::SystemPtr system_;
    Spinnaker::CameraList cameraList_;
    Spinnaker::CameraPtr camera_;
    std::string frameID_;
    std::string cameraSerialNo_;
    double frameRate_ = 15;
    enum triggerModeValues
    {
        SOFTWARE,
        HARDWARE,
        CONTINOUS
    };
    triggerModeValues triggerMode_ = SOFTWARE;

    int exposureTime_ = 15; // in millisec
    int maxExposureTime_ = 20; // in millisec

    enum exposureModeValues
    {
        MANUAL,
        AUTOMATIC
    };
    exposureModeValues exposureMode_ = MANUAL;

    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameterCallBackHandle_;
    std::shared_ptr<rclcpp::ParameterEventHandler> parameterEventHandler_;
    rclcpp::TimerBase::SharedPtr swTriggerTimer_;

    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager_;
    sensor_msgs::msg::Image imageMsg_;
    sensor_msgs::msg::CameraInfo cameraInfoMsg_;
    std::string cameraInfoURL_;
    int qosDepth_{4};
    image_transport::CameraPublisher imagePublisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr captureImageSub_;
    Spinnaker::ImagePtr imagePtrS_;

    // Image time out for grabbing the next image
    uint64_t imageTimeout_ = 100;
    // std::shared_ptr<std::thread> getImageThread_;
    rclcpp::Time captureTime_;

    Spinnaker::ImageProcessor imgProc_;

    /********************* Member Function *********************/
    void publishImage(Spinnaker::ImagePtr img);
    void declareParameters();
    void readParameters();
    bool initiliazeTheCamera();
    bool setCameraTriggerMode(triggerModeValues mode);
    std::string getCameraSerialNo(Spinnaker::CameraPtr cam);
    void softwareTriggerCamera();
    void synchronisedImageCapture(const std_msgs::msg::Float64::SharedPtr msg);
    void getImage();
    void setExposure();
    void setPixelFormat();
    void configureCamera(); 


    //rcl_interfaces::msg::SetParametersResult parameterChangeEvent(const std::vector<rclcpp::Parameter>& p);

    
};