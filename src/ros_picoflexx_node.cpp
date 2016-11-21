/*
 * Copyright (c) 2015, Mina  Kamel, ASL, ETH Zurich, Switzerland
 * You can contact the author at <mina.kamel@mavt.ethz.ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * created on Sep 30, 2015 5:00:11 AM
 */

#include <royale.hpp>
#include <iostream>
#include <ros_picoflexx/ros_picoflexx.h>
#include <memory>
#define NO_COLOR        "\033[0m"
#define FG_BLACK        "\033[30m"
#define FG_RED          "\033[31m"
#define FG_GREEN        "\033[32m"
#define FG_YELLOW       "\033[33m"
#define FG_BLUE         "\033[34m"
#define FG_MAGENTA      "\033[35m"
#define FG_CYAN         "\033[36m"
#define OUT_FUNCTION(NAME) ([](const std::string &name)\
{ \
  size_t end = name.rfind('(');\
  if(end == std::string::npos) end = name.size();\
  size_t begin = 1 + name.rfind(' ', end);\
  return name.substr(begin, end - begin);\
}(NAME))
#define OUT_AUX(FUNC_COLOR, MSG_COLOR, STREAM, MSG) STREAM(FUNC_COLOR "[" << OUT_FUNCTION(__PRETTY_FUNCTION__) << "] " MSG_COLOR << MSG << NO_COLOR)
#define OUT_INFO(msg) OUT_AUX(FG_GREEN, NO_COLOR, ROS_INFO_STREAM, msg)

PicoFlexxCamera::PicoFlexxCamera(royale::CameraManager& _manager, std::string _camera_name)
    : manager_(_manager),
      camera_name_(_camera_name)
{

}

PicoFlexxCamera::~PicoFlexxCamera()
{

}

bool PicoFlexxCamera::createCameraInfo(PicoFlexxCamera::DepthDataListener &listener, const royale::LensParameters &params)
{
  if(params.distortionRadial.size() != 3)
  {
    ROS_ERROR("distortion model unknown!");
    return false;
  }

  listener.camera_info_.height = camera_device_->getMaxSensorHeight();
  listener.camera_info_.width = camera_device_->getMaxSensorWidth();

  listener.camera_info_.K[0] = params.focalLength.first;
  listener.camera_info_.K[1] = 0;
  listener.camera_info_.K[2] = params.principalPoint.first;
  listener.camera_info_.K[3] = 0;
  listener.camera_info_.K[4] = params.focalLength.second;
  listener.camera_info_.K[5] = params.principalPoint.second;
  listener.camera_info_.K[6] = 0;
  listener.camera_info_.K[7] = 0;
  listener.camera_info_.K[8] = 1;

  listener.camera_info_.R[0] = 1;
  listener.camera_info_.R[1] = 0;
  listener.camera_info_.R[2] = 0;
  listener.camera_info_.R[3] = 0;
  listener.camera_info_.R[4] = 1;
  listener.camera_info_.R[5] = 0;
  listener.camera_info_.R[6] = 0;
  listener.camera_info_.R[7] = 0;
  listener.camera_info_.R[8] = 1;

  listener.camera_info_.P[0] = params.focalLength.first;
  listener.camera_info_.P[1] = 0;
  listener.camera_info_.P[2] = params.principalPoint.first;
  listener.camera_info_.P[3] = 0;
  listener.camera_info_.P[4] = 0;
  listener.camera_info_.P[5] = params.focalLength.second;
  listener.camera_info_.P[6] = params.principalPoint.second;
  listener.camera_info_.P[7] = 0;
  listener.camera_info_.P[8] = 0;
  listener.camera_info_.P[9] = 0;
  listener.camera_info_.P[10] = 1;
  listener.camera_info_.P[11] = 0;

  listener.camera_info_.distortion_model = "plumb_bob";
  listener.camera_info_.D.resize(5);
  listener.camera_info_.D[0] = params.distortionRadial[0];
  listener.camera_info_.D[1] = params.distortionRadial[1];
  listener.camera_info_.D[2] = params.distortionTangential.first;
  listener.camera_info_.D[3] = params.distortionTangential.second;
  listener.camera_info_.D[4] = params.distortionRadial[2];
  const royale::Vector<royale::Pair<royale::String,royale::String>> &info = camera_device_->getCameraInfo();
  for(size_t i = 0; i < info.size(); ++i)
  {
    OUT_INFO("  " << info[i].first << ": " FG_CYAN << info[i].second << NO_COLOR);
  }
  return true;
}

void PicoFlexxCamera::Initialize()
{
  //read ros parameters
  nh_ = ros::NodeHandle("~/" + camera_name_);

  nh_.param < std::string > ("camera_id", camera_id_, "0005-1206-0034-0815");
  nh_.param<int>("exposure_time", exposure_time_, 2000);
  nh_.param<bool>("auto_exposure_time", auto_exposure_time_, true);
  nh_.param <int> ("use_case", use_case_, 0);

  royale::Vector < royale::String > camlist = manager_.getConnectedCameraList();
  bool camera_initialized = false;
  for (int i = 0; i < camlist.size(); i++) {
    if (camlist[i] == camera_id_) {
      camera_initialized = true;
      camera_device_ = manager_.createCamera(camlist[i]);
      ROS_INFO_STREAM(camera_name_ << " initialized correctly");
    }
  }
  
  if(camera_initialized == false){
      ROS_ERROR("Camera ID cannot be found");
      return;
  }

  if (camera_device_->initialize() != royale::CameraStatus::SUCCESS) {
    ROS_ERROR("Cannot initialize the camera device");
    return;
  }
  camera_device_->getLensParameters (lensParams_);

  // display some information about the connected camera
  std::cout << "====================================" << std::endl;
  std::cout << "        Camera information" << std::endl;
  std::cout << "====================================" << std::endl;
  std::cout << "Id:              " << camera_device_->getId() << std::endl;
  std::cout << "Name:            " << camera_device_->getCameraName() << std::endl;
  std::cout << "Width:           " << camera_device_->getMaxSensorWidth() << std::endl;
  std::cout << "Height:          " << camera_device_->getMaxSensorHeight() << std::endl;
  std::cout << "Use Cases: " << camera_device_->getUseCases().size() << std::endl;
  std::cout << "Focal Length  fx: " << lensParams_.focalLength.first << std::endl;
  std::cout << "Focal Length  fy: " << lensParams_.focalLength.second << std::endl;
  std::cout << "Principal Pt. px: " << lensParams_.principalPoint.first << std::endl;
  std::cout << "Principal Pt. py: " << lensParams_.principalPoint.second << std::endl;
  std::cout << "Dist. Coeff k1:   " << lensParams_.distortionRadial[0] << std::endl;
  std::cout << "Dist. Coeff k2:   " << lensParams_.distortionRadial[1] << std::endl;
  std::cout << "Dist. Coeff k3:   " << lensParams_.distortionRadial[2] << std::endl;
  std::cout << "Dist. Coeff p1:   " << lensParams_.distortionTangential.first << std::endl;
  std::cout << "Dist. Coeff p2:   " << lensParams_.distortionTangential.second << std::endl;

//  for (auto mode : camera_device_->getUseCases()) {
//    std::cout << "    " << royale::getUseCaseName(mode) << std::endl;
//  }

  // set an operation mode
  //updateOperationMode();

  //set exposure settings
  updateExposureSettings();

  config_service_ = nh_.advertiseService("config", &PicoFlexxCamera::configCameraCallback, this);

  thread_ = std::thread(&PicoFlexxCamera::startAcquisition, this);
}

bool PicoFlexxCamera::startAcquisition()
{
  DepthDataListener listener(camera_name_);
  if (!createCameraInfo(listener, lensParams_)){
      ROS_ERROR("Could not create camera info");
      return false;
  }
  camera_device_->registerDataListener(&listener);

  // start capture mode
  camera_device_->startCapture();

  ros::waitForShutdown();

  if (camera_device_->isCapturing()) {
    camera_device_->stopCapture();
    std::cout << "Stop capturing..." << std::endl;
  }
  camera_device_->unregisterDataListener();
  camera_device_->stopRecording();
}

//void PicoFlexxCamera::updateOperationMode()
//{
//  royale::CameraStatus operation_mode_set = royale::CameraStatus::USECASE_NOT_SUPPORTED;
//  for (auto mode : camera_device_->getUseCases()) {
//    if (royale::getOperationModeName(mode) == use_case_) {
//      ROS_INFO_STREAM(" Setting Operaton Mode to:   " << royale::getOperationModeName(mode));
//      operation_mode_set = camera_device_->setUseCase(mode);
//    }
//  }

//  if (operation_mode_set != royale::CameraStatus::SUCCESS) {
//    ROS_WARN("Operation mode not supported. Setting default operation mode.");
//  }
//}

void PicoFlexxCamera::updateExposureSettings()
{

  royale::CameraStatus status;
  status = camera_device_->setExposureMode(royale::ExposureMode(auto_exposure_time_));

  if (status != royale::CameraStatus::SUCCESS) {
    ROS_WARN_STREAM("Exposure mode error : " << (int) status);
    return;
  }
  if (camera_device_->getExposureMode() == royale::ExposureMode::AUTOMATIC) {
    ROS_INFO("Automatic exposure time is selected.");
  } else {
    ROS_INFO("Manual exposure time is selected.");
    status = camera_device_->setExposureTime((uint32_t) exposure_time_);
    if (status != royale::CameraStatus::SUCCESS) {
      ROS_WARN_STREAM("Exposure mode error : " << (int) status);
    }
  }
}

bool PicoFlexxCamera::configCameraCallback(ros_picoflexx::PicoFlexxConfig::Request &req,
                                           ros_picoflexx::PicoFlexxConfig::Response &res)
{
  use_case_ = req.use_case;
  //updateOperationMode();
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "picoflexx");

  ros::NodeHandle nh("");
  royale::CameraManager manager;

  std::vector < std::shared_ptr < PicoFlexxCamera >> picoflexx_cameras_vector;

  //get cameras list
  royale::Vector <royale::String> camlist;
  int attempts = 5;

  ROS_INFO("Detecting cameras...");

  do {
    ros::Duration(1).sleep();
    camlist = manager.getConnectedCameraList();
    attempts--;
  } while (camlist.empty() & attempts > 0);

  std::cout << "Detected " << camlist.size() << " camera(s)." << std::endl;

  if (camlist.empty()) {
    ROS_ERROR("No PicoFLexx cameras detected.. aborting");
    return 1;
  }

  for (int i = 0; i < camlist.size(); i++) {
    std::cout << "Detected a camera with ID: " << camlist[i] << std::endl;
    std::shared_ptr<PicoFlexxCamera> tmp;
    picoflexx_cameras_vector.push_back(tmp);
    picoflexx_cameras_vector.back().reset(new PicoFlexxCamera(manager, "cam" + std::to_string(i)));

    picoflexx_cameras_vector.back()->Initialize();
  }

  ros::AsyncSpinner spinner(4);  // Use 4 threads
  spinner.start();
  ros::waitForShutdown();

}

