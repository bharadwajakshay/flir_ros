#include "flir_ros/flir_ros.hpp"

flirROS::flirROS(ros::NodeHandle node, ros::NodeHandle pnh):
    infoManager_(new camera_info_manager::CameraInfoManager(node)),
    it_(new image_transport::ImageTransport(node)){
    ROS_INFO("Entering constructor");

    // define params 
    this->declareParameters(pnh);
    
    // read params
    this->readParameters(pnh);

    bool camInitialization =  this->initiliazeTheCamera();

    if (!camInitialization){
        ROS_ERROR("The specified camera was not found. Killing the ROS driver");
        /************************** TODO *******************/
        // display all the serial numbers of all the cameras found
        exit(-1);
    }

    bool camTriggerMode = this->setCameraTriggerMode(triggerMode_);

    if(!camTriggerMode){
        ROS_ERROR("Error setting the trigger mode of the camera");
        exit(0);
    }

    this->configureCamera();

    if(triggerMode_ == CONTINOUS){
        /*****************************************************************************
         * Setting up ros timer to trigger the cameras using software to auto run the 
         * camera from a timer and not wait for the external trigger
         * ***************************************************************************/
        ROS_INFO("Setting up Trigger timer to a frame rate of %d FPS with a duration %lf", (int)this->frameRate_,(1/this->frameRate_));
        float duration = 1/this->frameRate_;
        ROS_INFO("CAlculated duration for timer %f",duration);
        this->triggerTimer = node.createTimer(ros::Duration(duration),&flirROS::triggerTimerCallback, (flirROS*)this);
    }

    if(this->infoManager_->validateURL(cameraInfoURL_)){
        ROS_INFO("CameraInforManager URL validate");
        this->infoManager_->loadCameraInfo(cameraInfoURL_);
    }

    this->cameraInfoMsg_ =  this->infoManager_->getCameraInfo();

    //image_transport::ImageTransport img_transport(node);

    //this->imagePublisher_ = img_transport.advertise("image_raw", 1);
    this->imagePublisher_ = it_->advertiseCamera("image_raw", 1);
    this->triggerSubscriber = node.subscribe("trigger", 10,&flirROS::synchronisedImageCapture,
                            (flirROS*)this,ros::TransportHints().tcpNoDelay(true));
}

void flirROS::readParameters(ros::NodeHandle& node){

    if(node.getParam("frameID", this->frameID_))
        ROS_INFO("Successfully retrived the param 'frameID': %s",this->frameID_.c_str());

    if(node.getParam("cameraSerialNo", this->cameraSerialNoInt_)){
        ROS_INFO("Successfully retrived the param 'cameraSerialNo': %d",this->cameraSerialNoInt_);
        this->cameraSerialNo_ = std::to_string(this->cameraSerialNoInt_);
    }
    else{
        ROS_ERROR("Failed to retrived the param 'cameraSerialNo'. Current Value is %d",this->cameraSerialNoInt_);
    }

    if(node.getParam("exposureMode", this->exposureModeStr_))
        ROS_INFO("Successfully retrived the param 'exposureMode': %s",this->exposureModeStr_.c_str());
    else 
        ROS_ERROR("Failed to retrived the param 'exposureMode'. Current Value is %s",this->exposureModeStr_.c_str());

    if(node.getParam("frameRate", this->frameRate_))
        ROS_INFO("Successfully retrived the param 'frameRate': %f",this->frameRate_);
    else 
        ROS_ERROR("Failed to retrived the param 'frameRate'. Current Value is %f",this->frameRate_);

    node.getParam("exposureTime", this->exposureTime_);
    node.getParam("maxExposureTime", this->maxExposureTime_);

    if(node.getParam("triggerMode", this->triggerModeStr_))
        ROS_INFO("Successfully retrived the param 'triggerMode': %s",this->triggerModeStr_.c_str());
    else 
        ROS_ERROR("Failed to retrived the param 'triggerMode'. Current Value is %s",this->triggerModeStr_.c_str());


    if(this->triggerModeStr_ == "continous")
        this->triggerMode_ = CONTINOUS;
    else if(this->triggerModeStr_ == "software")
        this->triggerMode_ = SOFTWARE;
    else if(this->triggerModeStr_ == "hardware")
        this->triggerMode_ == HARDWARE;
    else{
        ROS_ERROR("Undefined option for trigger: %s. Resorting to continous trigger.",this->triggerModeStr_.c_str());
        this->triggerMode_ = CONTINOUS;
    }

    if(this->exposureModeStr_ == "auto")
        this->exposureMode_ = AUTOMATIC;
    else if(this->exposureModeStr_ == "manual")
        this->exposureMode_ = MANUAL;
    else{
        ROS_ERROR("Undefined option for ExposureMode: %s. Resorting to Automatic trigger.",this->exposureModeStr_.c_str());
        this->exposureMode_ = AUTOMATIC;
    }

}

void flirROS::declareParameters(ros::NodeHandle& node){
    node.param<std::string>("frameID", this->frameID_, "flir_ros");
    node.param("cameraSerialNo", this->cameraSerialNoInt_, 12345);
    node.param<std::string>("exposureMode", this->exposureModeStr_, "auto");
    node.param("frameRate", this->frameRate_, 15.0);
    node.param("exposureTime", this->exposureTime_, 50);
    node.param("maxExposureTime", this->maxExposureTime_, 50);
    node.param<std::string>("triggerMode", this->triggerModeStr_, "continous");
    
}

bool flirROS::initiliazeTheCamera(){
    // get number of cameras connected to the system
    // get camera singleton
    this->system_ = Spinnaker::System::GetInstance();
    this->cameraList_ = system_->GetCameras();
    int noCameras = cameraList_.GetSize();
    ROS_INFO("The no of cameras conencted to the system are %d", noCameras);

    std::string sn;
    // iterate through all the cameras to find the camera with the correct serial number
    for (int camIdx=0; camIdx<noCameras; camIdx++ ){
        auto cam = this->cameraList_.GetByIndex(camIdx);
        sn = this->getCameraSerialNo(cam);
        if (sn == this->cameraSerialNo_){
            ROS_INFO("Found camera with serial number %s",sn.c_str());
            this->camera_ =  cam;
            this->camera_->Init();
            return true;
        }
    }
    ROS_ERROR("No camera with serial number %s found!!",this->cameraSerialNo_.c_str()); 
    return false;  

}

std::string flirROS::getCameraSerialNo(Spinnaker::CameraPtr cam)
{
  const auto & nodeMap = cam->GetTLDeviceNodeMap();
  const Spinnaker::GenApi::CStringPtr psn =
    nodeMap.GetNode("DeviceSerialNumber");
  return (Spinnaker::GenApi::IsAvailable(psn) && Spinnaker::GenApi::IsReadable(psn)) ? std::string(psn->GetValue()) : "";
}


bool flirROS::setCameraTriggerMode(triggerModeValues mode){
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
                                ROS_ERROR("Unable to set the trigger to SOFTWARE mode");
                        }
                        else if (mode == HARDWARE) {
                            Spinnaker::GenApi::CEnumEntryPtr ptrTriggerSourceSoftware = ptrTriggerSource->GetEntryByName("Line0");
                            if (Spinnaker::GenApi::IsAvailable(ptrTriggerSourceSoftware) && 
                                Spinnaker::GenApi::IsReadable(ptrTriggerSourceSoftware)){

                                ptrTriggerSource->SetIntValue(ptrTriggerSourceSoftware->GetValue());
                            }
                            else
                                ROS_ERROR("Unable to set the trigger to HARDWARE mode");

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
                                ROS_ERROR("Unable to begin the stream. Spinnaker exception. %s",e.what());
                                return false;
                            }

                            return true;
                        }
                        else
                            ROS_ERROR("Unable get the NODE to turn ON trigger");
                    }
                    else
                        ROS_ERROR("Unable get the NODE to select Trigger Source");
                }
                else
                    ROS_ERROR("Unable get the NODE to select Frame starter");
            }
            else
                ROS_ERROR("Unable get the NODE to select Trigger selector");
        }
        else
            ROS_ERROR("Unable get the NODE to turn off trigger");
    }
    else
        ROS_ERROR("Unable get the NODE to enable Trigegr Mode");
  return false;
}

void flirROS::configureCamera(){
    ROS_INFO("Configuring the camera");
    this->setExposure();
    this->setPixelFormat();
}

void flirROS::softwareTriggerCamera(){
    if (!camera_){
        ROS_ERROR("No camera is running. Unable to trigger an image.\nExiting!!!");
        exit(-1);
    }
    Spinnaker::GenApi::INodeMap & nodeMap = camera_->GetNodeMap();
    Spinnaker::GenApi::CCommandPtr ptrSWTrigger = nodeMap.GetNode("TriggerSoftware");

    if(Spinnaker::GenApi::IsWritable(ptrSWTrigger)){
      ptrSWTrigger->Execute();
      captureTime_ = ros::Time::now();
      std::thread getImageThread(&flirROS::getImage,this);
      getImageThread.detach();
    }
    else{
        ROS_ERROR("Failed to trigger an image.\nExiting!!!");
        exit(-1);
    }
}

// This is called by a thead from softwareTrigeerCamera()
void flirROS::getImage(){
    try{
        imagePtrS_ = camera_->GetNextImage(this->imageTimeout_); // timeout
        if(imagePtrS_->IsIncomplete()){
            ROS_ERROR("Captured image is incomplete");
            return;
        }
        std::thread publishImageThread(&flirROS::publishImage,this,imagePtrS_);
        publishImageThread.detach();
    }
    catch (Spinnaker::Exception& e){
        std::cout<< "Error: " << e.what() << std::endl;
        return;
    }
    
}

// This is called from the get image thread
void flirROS::publishImage(Spinnaker::ImagePtr imgPtr){

    Spinnaker::ImagePtr convertedImgPtr = imgProc_.Convert(imgPtr,Spinnaker::PixelFormatEnums::PixelFormat_BGR8);

    if(this->imagePublisher_.getNumSubscribers() >0){

        // setup cameraInfo 
        sensor_msgs::CameraInfo::Ptr cameraInfo(new sensor_msgs::CameraInfo(cameraInfoMsg_));
        sensor_msgs::Image::Ptr img(new sensor_msgs::Image(imageMsg_));

        bool fillStatus = sensor_msgs::fillImage(*img, sensor_msgs::image_encodings::BGR8,
          convertedImgPtr->GetHeight(), convertedImgPtr->GetWidth(),
          convertedImgPtr->GetStride(), convertedImgPtr->GetData());
        
        if (!fillStatus)
            ROS_ERROR("Error filling the Image to publish");

        else{
            img->header.frame_id = this->frameID_;
            img->header.stamp = this->captureTime_;
            this->imagePublisher_.publish(std::move(img),std::move(cameraInfo));
        }
    }

}

void flirROS::synchronisedImageCapture(const std_msgs::Bool::Ptr msg){

    ROS_DEBUG("Recieved the trigger. About to trigger the camera");
    this->softwareTriggerCamera();

}

void flirROS::triggerTimerCallback(const ros::TimerEvent& event){
    ROS_DEBUG("Recieved the trigger from trigger timer");
    this->softwareTriggerCamera();
}