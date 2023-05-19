#include <flir_ros2/flir_ros2.hpp>

void flirROS2::setExposure(){
    Spinnaker::GenApi::INodeMap &nodeMap = this->camera_->GetNodeMap();

    if(this->exposureMode_ == AUTOMATIC){
        Spinnaker::GenApi::CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
        if (Spinnaker::GenApi::IsReadable(ptrExposureAuto) && Spinnaker::GenApi::IsWritable(ptrExposureAuto)){
            Spinnaker::GenApi::CEnumEntryPtr ptrExposureAutoOn = ptrExposureAuto->GetEntryByName("Continuous");
            if(Spinnaker::GenApi::IsReadable(ptrExposureAutoOn)){
                ptrExposureAuto->SetIntValue(ptrExposureAutoOn->GetValue());
            }
        }
        else
            RCLCPP_ERROR(this->get_logger(),"Failed to set Exposure to automatic mode");

        // to avoid motion blur set the maximum exposure time
        Spinnaker::GenApi::CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
        if (!Spinnaker::GenApi::IsReadable(ptrExposureTime)){
            RCLCPP_ERROR(this->get_logger(),"Failed to get Max Exposure value for the camera");
            return;
        }

        const double maxAllowedExposureTime = ptrExposureTime->GetMax();
        const double maxExposureTime = ((this->maxExposureTime_*100.0) > maxAllowedExposureTime) ? maxAllowedExposureTime : this->maxExposureTime_*100.0;

        Spinnaker::GenApi::CFloatPtr ptrMaxExposureTime = nodeMap.GetNode("AutoExposureExposureTimeUpperLimit");
        if (Spinnaker::GenApi::IsReadable(ptrMaxExposureTime) && Spinnaker::GenApi::IsWritable(ptrMaxExposureTime)){
            ptrMaxExposureTime->SetValue(maxExposureTime);
        }
        else
            RCLCPP_ERROR(this->get_logger(),"Failed to set upperLimit for auto exposure");

    }
    else{

        Spinnaker::GenApi::CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
        const double maxAllowedExposureTime = ptrExposureTime->GetMax();
        const double exposureTime = ((this->exposureTime_*100.0) > maxAllowedExposureTime) ? maxAllowedExposureTime : this->exposureTime_*100.0;
        if (Spinnaker::GenApi::IsReadable(ptrExposureTime) && Spinnaker::GenApi::IsWritable(ptrExposureTime))
            ptrExposureTime->SetValue(exposureTime);
        else
            RCLCPP_ERROR(this->get_logger(),"Failed to set exposure value to %f",exposureTime);
    }
}

void flirROS2::setPixelFormat(){
    
}