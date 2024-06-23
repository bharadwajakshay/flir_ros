#include <memory>
#include <sstream>
#include <string>
#include <thread>

// Spinnaker SDK
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <std_msgs/Bool.h>

// OpenCV 
#include <opencv2/opencv.hpp>

// Header generated by dynamic_Reconfigure
#include <flir_ros/cameraConfig.h>
#include <dynamic_reconfigure/server.h>

class flirROS{
    public:
        flirROS(ros::NodeHandle node, ros::NodeHandle pnh); 

    private:
        ros::Publisher imagePublisher;
        ros::Subscriber triggerSubscriber;

        ros::Timer triggerTimer;
        
        Spinnaker::SystemPtr system_;
        Spinnaker::CameraList cameraList_;
        Spinnaker::CameraPtr camera_;
        Spinnaker::ImagePtr imagePtrS_;
        Spinnaker::ImageProcessor imgProc_;

        // Image time out for grabbing the next image
        uint64_t imageTimeout_ = 100;

        std::string frameID_;
        std::string cameraSerialNo_;
        int cameraSerialNoInt_;
        double frameRate_ = 15.0;

        enum triggerModeValues
        {
            SOFTWARE,
            HARDWARE,
            CONTINOUS
        };
        triggerModeValues triggerMode_ = SOFTWARE;
        std::string triggerModeStr_;

        int exposureTime_ = 15; // in millisec
        int maxExposureTime_ = 40; // in millisec

        enum exposureModeValues
        {
            MANUAL,
            AUTOMATIC
        };

        exposureModeValues exposureMode_ = MANUAL;
        std::string exposureModeStr_;

        std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager_;
        sensor_msgs::Image imageMsg_;
        sensor_msgs::CameraInfo cameraInfoMsg_;
        std::string cameraInfoURL_;
        image_transport::CameraPublisher imagePublisher_;
        std::shared_ptr<image_transport::ImageTransport> it_;
        ros::Time captureTime_;

        void publishImage(Spinnaker::ImagePtr img);
        void declareParameters(ros::NodeHandle& node);
        void readParameters(ros::NodeHandle& node);
        bool initiliazeTheCamera();
        bool setCameraTriggerMode(triggerModeValues mode);
        std::string getCameraSerialNo(Spinnaker::CameraPtr cam);
        void softwareTriggerCamera();
        void synchronisedImageCapture(const std_msgs::Bool::Ptr msg);
        void getImage();
        void setExposure();
        void setPixelFormat();
        void configureCamera();
        void triggerTimerCallback(const ros::TimerEvent& event);

};
