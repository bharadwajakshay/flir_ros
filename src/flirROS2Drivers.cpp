#include <flir_ros2/flir_ros2.hpp>

flirROS2::flirROS2():Node("flirDrivers"){
    RCLCPP_INFO(this->get_logger(),"Starting the constructor");

    // get parameters
    this->declareParameters();

    // read parameters
    this->readParameters();

    // Create a parameter change handler
    /**********************************ToD0********************************/
    /*parameterEventHandler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    //parameterCallBackHandle_ = this->add_on_set_parameters_callback(
    //    std::bind(&flirROS2::parameterChangeEvent,this,std::placeholders::_1));
    */

    bool camInitialization =  this->initiliazeTheCamera();

    if (!camInitialization){
        RCLCPP_ERROR(this->get_logger(),"The specified camera was not found. Killing the ROS driver");
        /************************** TODO *******************/
        // display all the serial numbers of all the cameras found
        exit(0);
    }

    bool camTriggerMode = this->setCameraTriggerMode(triggerMode_);

    if(!camTriggerMode){
        RCLCPP_ERROR(this->get_logger(),"Error setting the trigger mode of the camera");
        exit(0);
    }

    this->configureCamera();

    // Setup timer to run the software trigger
    if(triggerMode_ == CONTINOUS){
        swTriggerTimer_ = rclcpp::create_timer(this,get_clock(),rclcpp::Duration(0,static_cast<uint32_t>((1/frameRate_)*1e9)),
            std::bind(&flirROS2::softwareTriggerCamera,this));
    }

    /**************** ROS RELATED Declarations ******************/
    
    // Setup publisher
    // setup subscriber
    // setup camera info manager
    this->infoManager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, get_name(), cameraInfoURL_);
    cameraInfoMsg_ = infoManager_->getCameraInfo();

    rmw_qos_profile_t qosProf = rmw_qos_profile_default;
    qosProf.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qosProf.depth = qosDepth_;  // keep at most this number of images

    qosProf.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    // qosProf.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    // qosProf.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    // qosProf.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    qosProf.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;  // sender does not have to store
    qosProf.deadline.sec = 5;              // max expect time between msgs pub
    qosProf.deadline.nsec = 0;

    qosProf.lifespan.sec = 1;  // how long until msg are considered expired
    qosProf.lifespan.nsec = 0;

    qosProf.liveliness_lease_duration.sec = 10;  // time to declare client dead
    qosProf.liveliness_lease_duration.nsec = 0;

    this->imagePublisher_ = image_transport::create_camera_publisher(this,"~/image_raw",qosProf);
    this->captureImageSub_ = this->create_subscription<std_msgs::msg::Float64>("testCamera",this->frameRate_,
        std::bind(&flirROS2::synchronisedImageCapture,this,std::placeholders::_1));
    
}

void flirROS2::configureCamera(){
    RCLCPP_INFO(this->get_logger(),"Configuring the camera");
    this->setExposure();
    this->setPixelFormat();
}

void flirROS2::synchronisedImageCapture(const std_msgs::msg::Float64::SharedPtr msg){

}

void flirROS2::softwareTriggerCamera(){
    if (!camera_){
        RCLCPP_ERROR(this->get_logger(),"No camera is running. Unable to trigger an image.\nExiting!!!");
        exit(0);
    }
    Spinnaker::GenApi::INodeMap & nodeMap = camera_->GetNodeMap();
    Spinnaker::GenApi::CCommandPtr ptrSWTrigger = nodeMap.GetNode("TriggerSoftware");

    if(Spinnaker::GenApi::IsWritable(ptrSWTrigger)){
      ptrSWTrigger->Execute();
      captureTime_ = this->get_clock()->now();
      std::thread getImageThread(&flirROS2::getImage,this);
      getImageThread.detach();
    }
    else{
        RCLCPP_ERROR(this->get_logger(),"Failed to trigger an image.\nExiting!!!");
        exit(0);
    }
}

// This is called by a thead from softwareTrigeerCamera()
void flirROS2::getImage(){
    try{
        imagePtrS_ = camera_->GetNextImage(this->imageTimeout_); // timeout
        if(imagePtrS_->IsIncomplete()){
            RCLCPP_ERROR(this->get_logger(),"Captured image is incomplete");
            return;
        }
        std::thread publishImageThread(&flirROS2::publishImage,this,imagePtrS_);
        publishImageThread.detach();
    }
    catch (Spinnaker::Exception& e){
        std::cout<< "Error: " << e.what() << std::endl;
        return;
    }
    
}

// This is called from the get image thread
void flirROS2::publishImage(Spinnaker::ImagePtr imgPtr){

    Spinnaker::ImagePtr convertedImgPtr = imgProc_.Convert(imgPtr,Spinnaker::PixelFormatEnums::PixelFormat_BGR8);

    if(this->imagePublisher_.getNumSubscribers() >0){

        // setup cameraInfo 
        sensor_msgs::msg::CameraInfo::UniquePtr cameraInfo(new sensor_msgs::msg::CameraInfo(cameraInfoMsg_));
        sensor_msgs::msg::Image::UniquePtr img(new sensor_msgs::msg::Image(imageMsg_));

        bool fillStatus = sensor_msgs::fillImage(*img, sensor_msgs::image_encodings::BGR8,
          convertedImgPtr->GetHeight(), convertedImgPtr->GetWidth(),
          convertedImgPtr->GetStride(), convertedImgPtr->GetData());
        
        if (!fillStatus)
            RCLCPP_ERROR(this->get_logger(),"Error filling the Image to publish");

        else{
            img->header.frame_id = this->frameID_;
            img->header.stamp = this->captureTime_;
            this->imagePublisher_.publish(std::move(img),std::move(cameraInfo));
        }
    }

}

bool flirROS2::setCameraTriggerMode(triggerModeValues mode){
    // Get node Map
    Spinnaker::GenApi::INodeMap &nodeMap = camera_->GetNodeMap();
    // Turn Off the trigger mode
    Spinnaker::GenApi::CEnumerationPtr ptrTriggerMode = 
        nodeMap.GetNode("TriggerMode");
    if (Spinnaker::GenApi::IsAvailable(ptrTriggerMode) && 
        Spinnaker::GenApi::IsReadable(ptrTriggerMode)) {
    
        Spinnaker::GenApi::CEnumEntryPtr ptrTriggerModeOff 
            = ptrTriggerMode->GetEntryByName("Off");
        if (Spinnaker::GenApi::IsAvailable(ptrTriggerModeOff) && 
            Spinnaker::GenApi::IsReadable(ptrTriggerModeOff)) {

            ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());

            // Set trigger selector to FrameStart
            Spinnaker::GenApi::CEnumerationPtr ptrTiggerSelector =
                nodeMap.GetNode("TriggerSelector");
            if (Spinnaker::GenApi::IsAvailable(ptrTiggerSelector) && 
                Spinnaker::GenApi::IsReadable(ptrTiggerSelector) && 
                Spinnaker::GenApi::IsWritable(ptrTiggerSelector)) {
        
                Spinnaker::GenApi::CEnumEntryPtr ptrTiggerSelectorFrameStart = 
                    ptrTiggerSelector->GetEntryByName("FrameStart");
                
                if (Spinnaker::GenApi::IsAvailable(ptrTiggerSelectorFrameStart) &&
                    Spinnaker::GenApi::IsReadable(ptrTiggerSelectorFrameStart)) {

                    ptrTiggerSelector->SetIntValue(ptrTiggerSelectorFrameStart->GetValue());

                    // Set trigger to Software
                    Spinnaker::GenApi::CEnumerationPtr ptrTriggerSource =
                        nodeMap.GetNode("TriggerSource");
                    if (Spinnaker::GenApi::IsAvailable(ptrTriggerSource) && 
                        Spinnaker::GenApi::IsReadable(ptrTriggerSource) && 
                        Spinnaker::GenApi::IsWritable(ptrTriggerSource)) {

                               
                        if (mode==SOFTWARE){
                            Spinnaker::GenApi::CEnumEntryPtr ptrTriggerSourceSoftware = ptrTriggerSource->GetEntryByName("Software");
                            if (Spinnaker::GenApi::IsAvailable(ptrTriggerSourceSoftware) && 
                                Spinnaker::GenApi::IsReadable(ptrTriggerSourceSoftware)){

                            ptrTriggerSource->SetIntValue(ptrTriggerSourceSoftware->GetValue());
                            }
                            else
                                RCLCPP_ERROR(this->get_logger(),"Unable to set the trigger to SOFTWARE mode");
                        }
                        else if (mode == HARDWARE) {
                            Spinnaker::GenApi::CEnumEntryPtr ptrTriggerSourceSoftware = ptrTriggerSource->GetEntryByName("Line0");
                            if (Spinnaker::GenApi::IsAvailable(ptrTriggerSourceSoftware) && 
                                Spinnaker::GenApi::IsReadable(ptrTriggerSourceSoftware)){

                                ptrTriggerSource->SetIntValue(ptrTriggerSourceSoftware->GetValue());
                            }
                            else
                                RCLCPP_ERROR(this->get_logger(),"Unable to set the trigger to HARDWARE mode");

                        }

                        // Turn ON the trigger mode
                        Spinnaker::GenApi::CEnumEntryPtr ptrTriggerModeOn =
                            ptrTriggerMode->GetEntryByName("On");
                        if (Spinnaker::GenApi::IsAvailable(ptrTriggerModeOn) && 
                            Spinnaker::GenApi::IsReadable(ptrTriggerModeOn)){

                            ptrTriggerMode->SetIntValue(ptrTriggerModeOn->GetValue());

                            try{
                                // begin the stream
                                camera_->BeginAcquisition();
                            }
                            catch (Spinnaker::Exception &e){
                                RCLCPP_ERROR(this->get_logger(),"Unable to begin the stream. Spinnaker exception. %s",e.what());
                                return false;
                            }

                            return true;
                        }
                        else
                            RCLCPP_ERROR(this->get_logger(),"Unable get the NODE to turn ON trigger");
                    }
                    else
                        RCLCPP_ERROR(this->get_logger(),"Unable get the NODE to select Trigger Source");
                }
                else
                    RCLCPP_ERROR(this->get_logger(),"Unable get the NODE to select Frame starter");
            }
            else
                RCLCPP_ERROR(this->get_logger(),"Unable get the NODE to select Trigger selector");
        }
        else
            RCLCPP_ERROR(this->get_logger(),"Unable get the NODE to turn off trigger");
    }
    else
        RCLCPP_ERROR(this->get_logger(),"Unable get the NODE to enable Trigegr Mode");
  return false;
}

bool flirROS2::initiliazeTheCamera(){
    // get number of cameras connected to the system
    // get camera singleton
    this->system_ = Spinnaker::System::GetInstance();
    this->cameraList_ = system_->GetCameras();
    int noCameras = cameraList_.GetSize();
    RCLCPP_INFO(this->get_logger(),"The no of cameras conencted to the system are %d", noCameras);

    std::string sn;
    // iterate through all the cameras to find the camera with the correct serial number
    for (int camIdx=0; camIdx<noCameras; camIdx++ ){
        auto cam = this->cameraList_.GetByIndex(camIdx);
        sn = this->getCameraSerialNo(cam);
        if (sn == this->cameraSerialNo_){
            RCLCPP_INFO(this->get_logger(),"Found camera with serial number %s",sn.c_str());
            this->camera_ =  cam;
            this->camera_->Init();
            return true;
        }
    }
    RCLCPP_INFO(this->get_logger(),"No camera with serial number %s found!!",sn.c_str()); 
    return false;  

}

void flirROS2::declareParameters(){
    // Declare ROSparameters from launch
    this->declare_parameter("triggerMode","continous");
    this->declare_parameter("exposureMode","auto");
    this->declare_parameter("frameRate",15.5);
    this->declare_parameter("frameID","flirCamera");
    this->declare_parameter("exposureTime",15);
    this->declare_parameter("maxExposureTime",9);
    this->declare_parameter("cameraSerialNo","12345");
}

void flirROS2::readParameters(){
    RCLCPP_INFO(this->get_logger(),"Reading Parameters from the launch file");

    frameRate_ = this->get_parameter("frameRate").as_double();
    cameraSerialNo_ = this->get_parameter("cameraSerialNo").as_string();
    frameID_ = this->get_parameter("frameID").as_string();    
    if (this->get_parameter("triggerMode").as_string() == "continous")
        triggerMode_ = CONTINOUS;
    else if (this->get_parameter("triggerMode").as_string() == "software")
        triggerMode_ = SOFTWARE;
    else
        triggerMode_ = HARDWARE;
    if (this->get_parameter("exposureMode").as_string() == "auto")
        exposureMode_ = AUTOMATIC;
    else
        exposureMode_ = MANUAL;
    exposureTime_ = this->get_parameter("exposureTime").as_int();
    maxExposureTime_ = this->get_parameter("maxExposureTime").as_int();
}

std::string flirROS2::getCameraSerialNo(Spinnaker::CameraPtr cam)
{
  const auto & nodeMap = cam->GetTLDeviceNodeMap();
  const Spinnaker::GenApi::CStringPtr psn =
    nodeMap.GetNode("DeviceSerialNumber");
  return (Spinnaker::GenApi::IsAvailable(psn) && Spinnaker::GenApi::IsReadable(psn)) ? std::string(psn->GetValue()) : "";
}

