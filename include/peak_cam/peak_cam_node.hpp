// Copyright (c) 2020, Sherif Nekkah 
// All rights reserved. 
// 
// DISCLAMER:
//
//
// This package was created and used within an academic project and should
// be considered as experimental code. There may be bugs and deficiencies in the
// software. Feel free for suggestions, pull requests or any possible issue. 
//
//
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met: 
// 
//  * Redistributions of source code must retain the above copyright notice, 
//    this list of conditions and the following disclaimer. 
//  * Redistributions in binary form must reproduce the above copyright 
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the distribution. 
//  * Neither the name of  nor the names of its contributors may be used to 
//    endorse or promote products derived from this software without specific 
//    prior written permission. 
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE. 
#ifndef PEAK_CAM__PEAK_CAM_NODE_HPP_
#define PEAK_CAM__PEAK_CAM_NODE_HPP_

#include <iostream>
#include <atomic>
#include <memory>

//ROS Headers
#include "camera_info_manager/camera_info_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

//OpenCV Headers
#include "opencv2/opencv.hpp"
#if defined __has_include
#if __has_include("cv_bridge/cv_bridge.hpp")
#include "cv_bridge/cv_bridge.hpp"
#else
#include "cv_bridge/cv_bridge.h"
#endif
#else
#include "cv_bridge/cv_bridge.h"
#endif
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

//IDS Camera Headers
#include <peak_ipl/peak_ipl.hpp>
#include <peak/converters/peak_buffer_converter_ipl.hpp>
#include <peak/peak.hpp>

//Parameters
#include "peak_cam/acquisition_parameters.hpp"


namespace peak_cam
{

using peak::core::nodes::EnumerationNode;
using peak::core::nodes::FloatNode;
using peak::core::nodes::BooleanNode;
using peak::core::nodes::IntegerNode;

class PeakCamNode : public rclcpp::Node
{
public:
  explicit PeakCamNode(const rclcpp::NodeOptions & options);
  ~PeakCamNode();

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pubImage;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_pubCameraInfo;

  /// Camera Info Manager
  std::shared_ptr<camera_info_manager::CameraInfoManager> m_cameraInfoManager;

  std::shared_ptr<peak::core::DataStream> m_dataStream;
  std::shared_ptr<peak::core::Device> m_device;
  std::shared_ptr<peak::core::NodeMap> m_nodeMapRemoteDevice;

  peak::ipl::PixelFormatName m_inputPixelFormat;
  peak::ipl::PixelFormatName m_pixelFormat;
  
  std::string m_image_encoding;
  std_msgs::msg::Header::SharedPtr m_header;
  sensor_msgs::msg::CameraInfo::SharedPtr m_cameraInfo;

  rclcpp::TimerBase::SharedPtr m_acquisitionTimer;

  cv::Mat m_cvImage;
  cv_bridge::CvImagePtr m_cvImagePtr;
  std::unique_ptr<peak::ipl::ImageConverter> m_imageConverter;
  peak::ipl::ConversionMode conversionMode{peak::ipl::ConversionMode::Fast};

  // Camera Parameters
  Peak_Params m_peakParams;

  std::string m_frameId, m_imageTopic, m_cameraInfoUrl;
  uint8_t m_bytesPerPixel;

  // Preventing two threads to acces variable acquisitionLoopRunning
  std::atomic<bool> m_acquisitionLoopRunning{false};

  // acquisitionLoop function and bool are public to run on particular thread
  void acquisitionLoop();
  void getParams();
  void openDevice();
  void setDeviceParameters();
  void setPTPDeviceParameters();
  void closeDevice();

  // Various helper functions to help for the decalaration of parameters.
  template<typename T>
    void declareParameter(const std::string &name, const T& default_value, const std::string &description, T& storage);
  template<typename T>
    void declareParameter(const std::string &name, const T& default_value, T& storage);
  void declareParameter(const std::string &name, const char *default_value, const std::string &description, std::string& storage)
  {
    this->declareParameter(name, std::string(default_value), description, storage);
  }
  void declareParameter(const std::string &name, const char *default_value, std::string& storage)
  {
    this->declareParameter(name, std::string(default_value), storage);
  }

  // Various helper functions to set/get parameter values in the camera

  template<class N>
    bool remoteDeviceParameterExists(const std::string &node_name)
  {
    try {
      auto node __attribute__((unused)) = m_nodeMapRemoteDevice->FindNode<N>(node_name);
      return true;
    }
    catch (const peak::core::NotFoundException &) {
      return false;
    }
  }

  template<class N>
    bool remoteDeviceParameterExists(const char *node_name)
  {
    return remoteDeviceParameterExists<N>(std::string(node_name));
  }

  template<class N, typename V>
    bool setRemoteDeviceParameter(const std::string &node_name, const V& value);

  template<class N, typename V>
    V getRemoteDeviceParameter(const std::string &node_name)
  {
    try {
      auto node = m_nodeMapRemoteDevice->FindNode<N>(node_name);
      return getRemoteNodeValue(node);
    }
    catch (const peak::core::NotFoundException &e) {
      RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: %s is not a parameter for this camera", node_name.c_str());
      return V{};
    }
  }

  template<class N, typename V>
    V getRemoteDeviceParameter(const char *node_name)
  {
    try {
      auto node = m_nodeMapRemoteDevice->FindNode<N>(node_name);
      return getRemoteNodeValue(node);
    }
    catch (const peak::core::NotFoundException &e) {
      RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: %s is not a parameter for this camera", node_name);
      return V{};
    }
  }

  void setRemoteNodeValue(std::shared_ptr<peak::core::nodes::EnumerationNode> node, const std::string &value) { node->SetCurrentEntry(value); }
  void setRemoteNodeValue(std::shared_ptr<peak::core::nodes::FloatNode> node, const double value) { node->SetValue(value); }
  void setRemoteNodeValue(std::shared_ptr<peak::core::nodes::IntegerNode> node, const int64_t value) { node->SetValue(value); }
  void setRemoteNodeValue(std::shared_ptr<peak::core::nodes::BooleanNode> node, const bool value) { node->SetValue(value); }

  std::string getRemoteNodeValue(std::shared_ptr<peak::core::nodes::EnumerationNode> node) { return node->CurrentEntry()->SymbolicValue(); }
  double getRemoteNodeValue(std::shared_ptr<peak::core::nodes::FloatNode> node) { return node->Value(); }
  int64_t getRemoteNodeValue(std::shared_ptr<peak::core::nodes::IntegerNode> node) { return node->Value(); }
  bool getRemoteNodeValue(std::shared_ptr<peak::core::nodes::BooleanNode> node) { return node->Value(); }

  template<class N, typename V>
    V getRemoteDeviceParameterMaximum(const std::string &node_name)
  {
    try {
      auto node = m_nodeMapRemoteDevice->FindNode<N>(node_name);
      return getRemoteNodeMaximum<N, V>(node);
    }
    catch (const peak::core::NotFoundException &e) {
      RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: %s is not a parameter for this camera", node_name.c_str());
      return V{};
    }
  }

  template<class N, typename V>
    V getRemoteDeviceParameterMinimum(const std::string &node_name)
  {
    try {
      auto node = m_nodeMapRemoteDevice->FindNode<N>(node_name);
      return getRemoteNodeMinimum<N, V>(node);
    }
    catch (const peak::core::NotFoundException &e) {
      RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: %s is not a parameter for this camera", node_name.c_str());
      return V{};
    }
  }

  template<class N, typename V>
    V getRemoteDeviceParameterIncrement(const std::string &node_name)
  {
    try {
      auto node = m_nodeMapRemoteDevice->FindNode<N>(node_name);
      return getRemoteNodeIncrement<N, V>(node);
    }
    catch (const peak::core::NotFoundException &e) {
      RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: %s is not a parameter for this camera", node_name.c_str());
      return V{};
    }
  }

  template <class N, typename V>
    V getRemoteNodeMaximum(std::shared_ptr<N> node) { return node->Maximum(); }
  template <class N, typename V>
    V getRemoteNodeMinimum(std::shared_ptr<N> node) { return node->Minimum(); }
  template <class N, typename V>
    V getRemoteNodeIncrement(std::shared_ptr<N> node) { return node->Increment(); }

  void describeRemoteNodeRange(std::shared_ptr<EnumerationNode> node);
  void describeRemoteNodeRange(std::shared_ptr<FloatNode> node);
  void describeRemoteNodeRange(std::shared_ptr<IntegerNode> node);
  void describeRemoteNodeRange(std::shared_ptr<BooleanNode>) {}

};

} // namespace peak_cam

#endif  // PEAK_CAM__PEAK_CAM_NODE_HPP_
