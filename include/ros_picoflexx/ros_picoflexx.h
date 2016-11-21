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
 * created on Oct 1, 2015 7:54:43 AM
 */
#ifndef INCLUDE_ROS_PICOFLEXX_ROS_PICOFLEXX_H_
#define INCLUDE_ROS_PICOFLEXX_ROS_PICOFLEXX_H_
#include <royale.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <thread>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros_picoflexx/PicoFlexxConfig.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class PicoFlexxCamera
{
 public:
  PicoFlexxCamera(royale::CameraManager& _manager);
  ~PicoFlexxCamera();

  void Initialize();

  bool startAcquisition();

  //setters
  void setExposureTime(int _exposure_time);
  void setExposureTimeLimits(int _minimum_exposure_time, int _maximum_exposure_time);
  void setAutoExposureTime(bool _auto_exposure_time);
  void setFrameRate(int _frame_rate);
  //void setOperationMode(royale::OperationMode _operation_mode);
  void setUseCase(royale::String &useCase);
  royale::LensParameters lensParams_;

 private:
  royale::CameraManager& manager_;
  std::unique_ptr<royale::ICameraDevice> camera_device_;

  std::string camera_id_;
  std::string camera_name_;
  int exposure_time_;
  bool auto_exposure_time_;
  std::string frame_id_;
  int use_case_;
  ros::NodeHandle nh_;

  //void updateUseCase();
  void updateExposureSettings();

public:
  class DepthDataListener : public royale::IDepthDataListener
  {
   public:
    DepthDataListener(std::string _camera_name)
    {
      ros::NodeHandle nh(_camera_name);

      nh.param<std::string>("point_cloud_topic", point_cloud_topic_,  "points");
      nh.param<std::string>("frame_id", frame_id_, _camera_name+"_optical_frame");
      nh.param<std::string>("depth_image_topic", depth_image_topic_,  "image_depth");
      nh.param<std::string>("grey_image_topic", grey_image_topic_,  "image_mono8");
      nh.param<std::string>("camera_info_topic", camera_info_topic_, "camera_info");
      point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(point_cloud_topic_, 1);
      image_transport::ImageTransport image_transport(nh);
      depth_image_pub_ = image_transport.advertise(depth_image_topic_, 1);
      grey_image_pub_ = image_transport.advertise(grey_image_topic_, 1);
      camera_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>(camera_info_topic_, 1);

      seq_ = 0;
      cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
    void onNewData(const royale::DepthData *data)
    {

      cloud_->is_dense = false;
      cloud_->height = data->height;
      cloud_->width = data->width;
      cloud_->points.resize(cloud_->height * cloud_->width);
      cv::Mat depth_mat(data->height,data->width,CV_32FC1);
      cv::Mat grey_mat(data->height,data->width,CV_16UC1);

      for (int i = 0; i < data->points.size(); i++) {
        pcl::PointXYZ& pt = cloud_->points[i];
        grey_mat.at<uint16_t>(i/data->width, i%data->width) = data->points[i].grayValue;
        if (data->points[i].noise < 0.05 * data->points[i].z){
            pt.x = data->points[i].x;
            pt.y = data->points[i].y;
            pt.z = data->points[i].z;
            //depth_mat.at<unsigned short>(i/data->width, i%data->width) = (unsigned short)5000*data->points[i].z;
            depth_mat.at<float>(i/data->width, i%data->width) = data->points[i].z;
        }
        else{
            pt.x = bad_point_;
            pt.y = bad_point_;
            pt.z = bad_point_;
            depth_mat.at<float>(i/data->width, i%data->width) = bad_point_;
        }
      }

      pcl::toROSMsg(*cloud_, pcl2_msg_);

      pcl2_msg_.header.seq = seq_;
      pcl2_msg_.header.frame_id = frame_id_;
      pcl2_msg_.header.stamp.sec = (int) (data->timeStamp.count() / 1000);
      pcl2_msg_.header.stamp.nsec = (data->timeStamp.count()
          - (int) (data->timeStamp.count() / 1000) * 1000) * 1000000;

      point_cloud_pub_.publish(pcl2_msg_);
      sensor_msgs::CameraInfoPtr camera_info_msg_;
      camera_info_msg_ = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo);
      *camera_info_msg_ = camera_info_;
      camera_info_pub_.publish(camera_info_msg_);
      depth_img_.header = pcl2_msg_.header;
      depth_img_.width = depth_mat.cols;
      depth_img_.height = depth_mat.rows;
      depth_img_.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      depth_img_.is_bigendian = 0;
      int step = (uint32_t)(sizeof(float) * depth_img_.width);
      int size = step * depth_img_.height;
      depth_img_.step = step;
      depth_img_.data.resize(size);
      memcpy(&depth_img_.data[0], depth_mat.data, size);

      img_bridge_ = cv_bridge::CvImage(depth_img_.header, sensor_msgs::image_encodings::MONO16, grey_mat);
      img_bridge_.toImageMsg(grey_img_); // from cv_bridge to sensor_msgs::Image

      depth_image_pub_.publish(depth_img_);
      grey_image_pub_.publish(grey_img_);
      seq_++;
    }
   sensor_msgs::CameraInfo camera_info_;
   private:
    sensor_msgs::PointCloud2 pcl2_msg_;
    sensor_msgs::Image depth_img_;
    sensor_msgs::Image grey_img_;
    cv_bridge::CvImage img_bridge_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    int seq_;
    std::string point_cloud_topic_;
    std::string depth_image_topic_;
    std::string grey_image_topic_;
    std::string camera_info_topic_;
    std::string frame_id_;
    ros::Publisher point_cloud_pub_;
    ros::Publisher camera_info_pub_;
    const float bad_point_ = std::numeric_limits<float>::quiet_NaN();
    image_transport::Publisher depth_image_pub_;
    image_transport::Publisher grey_image_pub_;

  };
  bool createCameraInfo(PicoFlexxCamera::DepthDataListener &listener, const royale::LensParameters &params);
  std::thread thread_;
  ros::ServiceServer config_service_;
  bool configCameraCallback(ros_picoflexx::PicoFlexxConfig::Request  &req,
                            ros_picoflexx::PicoFlexxConfig::Response &res);

};

#endif /* INCLUDE_ROS_PICOFLEXX_ROS_PICOFLEXX_H_ */
