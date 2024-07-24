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
#include "peak_cam/peak_cam_node.hpp"
#include<iostream>
#include <cstdlib>
#include <unistd.h>

namespace peak_cam
{

PeakCamNode::PeakCamNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("peak_cam_node", options)
{
  getParams();

  m_pubImage = this->create_publisher<sensor_msgs::msg::Image>(std::string(this->get_name()) + "/" +  m_imageTopic, 1);
  m_pubCameraInfo =
    this->create_publisher<sensor_msgs::msg::CameraInfo>(std::string(this->get_name()) + "/camera_info", 1);
  
  // Initialize header messages
  m_header.reset(new std_msgs::msg::Header());
  m_header->frame_id = m_frameId;

  // Initialize Camera Info Manager
  m_cameraInfoManager =
    std::make_shared<camera_info_manager::CameraInfoManager>(
      this,
      m_frameId,
      m_cameraInfoUrl);

  m_cameraInfoManager->setCameraName(m_frameId);

  if(m_cameraInfoManager->validateURL(m_cameraInfoUrl)) {
    m_cameraInfoManager->loadCameraInfo(m_cameraInfoUrl);
  } else {
    RCLCPP_WARN(this->get_logger(), "The Provided Camera Info URL is invalid or file does not exist:");
    RCLCPP_WARN(this->get_logger(), "  %s", m_cameraInfoUrl.c_str());
    RCLCPP_WARN(this->get_logger(), "Uncalibrated Camera Info will be published...");
  }
  
  // set acqusition callback
  m_acquisitionTimer =
    this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&PeakCamNode::acquisitionLoop, this));
  
  peak::Library::Initialize();
  openDevice();
}

PeakCamNode::~PeakCamNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down");
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop")->Execute();
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop")->WaitUntilDone();
  // closing camera und peak library
  closeDevice();
  peak::Library::Close();
  RCLCPP_INFO(this->get_logger(), "Peak library closed");
}

void PeakCamNode::getParams()
{
  try {
    declare_parameter("frame_id", "defaultFrameId");
    get_parameter("frame_id", m_frameId);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The frame_id provided was invalid");
    throw ex;
  }
  
  try {
    declare_parameter("image_topic", "defaultImageTopic");
    get_parameter("image_topic", m_imageTopic);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The image_topic provided was invalid");
    throw ex;
  }

  try {
    declare_parameter("camera_info_url", "defaultCameraInfoUrl");
    get_parameter("camera_info_url", m_cameraInfoUrl);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The camera_info_url provided was invalid");
    throw ex;
  }

  try {
    declare_parameter("ExposureTime", 10000);
    get_parameter("ExposureTime", m_peakParams.ExposureTime);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The ExposureTime provided was invalid");
    throw ex;
  }
  
  try {
    declare_parameter("AcquisitionFrameRate", 30);
    get_parameter("AcquisitionFrameRate", m_peakParams.AcquisitionFrameRate);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The AcquisitionFrameRate provided was invalid");
    throw ex;
  }
  
  try {
    declare_parameter("ImageHeight", 1080);
    get_parameter("ImageHeight", m_peakParams.ImageHeight);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The ImageHeight provided was invalid");
    throw ex;
  }
  
  try {
    declare_parameter("ImageWidth", 1920);
    get_parameter("ImageWidth", m_peakParams.ImageWidth);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The ImageWidth provided was invalid");
    throw ex;
  }
  
  try {
    declare_parameter("UseOffset", false);
    get_parameter("UseOffset", m_peakParams.UseOffset);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The UseOffset provided was invalid");
    throw ex;
  }
  
  try {
    declare_parameter("OffsetHeight", 0);
    get_parameter("OffsetHeight", m_peakParams.OffsetHeight);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The OffsetHeight provided was invalid");
    throw ex;
  }
  
  try {
    declare_parameter("OffsetWidth", 0);
    get_parameter("OffsetWidth", m_peakParams.OffsetWidth);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The OffsetWidth provided was invalid");
    throw ex;
  }

  try {
    declare_parameter("Gamma", 1.0);
    get_parameter("Gamma", m_peakParams.Gamma);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The Gamma provided was invalid");
    throw ex;
  }

  try {
    declare_parameter("selectedDevice", "defaultDevice");
    get_parameter("selectedDevice", m_peakParams.selectedDevice);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The selectedDevice provided was invalid");
    throw ex;
  }

  try {
    declare_parameter("ExposureAuto", "Off");
    get_parameter("ExposureAuto", m_peakParams.ExposureAuto);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The ExposureAuto provided was invalid");
    throw ex;
  }

  try {
    declare_parameter("GainAuto", "Off");
    get_parameter("GainAuto", m_peakParams.GainAuto);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The GainAuto provided was invalid");
    throw ex;
  }
  
  try {
    declare_parameter("PixelFormat", "Mono8");
    get_parameter("PixelFormat", m_peakParams.PixelFormat);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The PixelFormat provided was invalid");
    throw ex;
  }
  
  try {
    declare_parameter("GainSelector", "All");
    get_parameter("GainSelector", m_peakParams.GainSelector);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The GainSelector provided was invalid");
    throw ex;
  }

  try {
    declare_parameter("TriggerSource", "Software");
    get_parameter("TriggerSource", m_peakParams.TriggerSource);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The TriggerSource provided was invalid");
    throw ex;
  }

  try {
    declare_parameter("TriggerActivation", "RisingEdge");
    get_parameter("TriggerActivation", m_peakParams.TriggerActivation);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The TriggerActivation provided was invalid");
    throw ex;
  }

  try {
    declare_parameter("DeviceLinkThroughputLimit", 1000000);
    get_parameter("DeviceLinkThroughputLimit", m_peakParams.DeviceLinkThroughputLimit);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "DeviceLinkThroughputLimit provided was invalid");
    throw ex;
  }

  try {
    declare_parameter("PixelClock", 20);
    get_parameter("PixelClock", m_peakParams.PixelClock);  // get it in MHz
    m_peakParams.PixelClock *= 1000 * 1000;   // Convert to Hz
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "PixelClock provided was invalid");
    throw ex;
  }

  try {
    declare_parameter("Line1Source", "defaultSource");
    get_parameter("Line1Source", m_peakParams.Line1Source);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "Line1Source provided was invalid");
    throw ex;
  }

  try {
    declare_parameter("TriggerDivider", 1);
    get_parameter("TriggerDivider", m_peakParams.TriggerDivider);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "TriggerDivider provided was invalid");
    throw ex;
  }

  try {
    declare_parameter("PtpEnable", false);
    get_parameter("PtpEnable", m_peakParams.PtpEnable);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "PtpEnable provided was invalid");
    throw ex;
  }

  try {
    declare_parameter("PtpSlaveOnly", false);
    get_parameter("PtpSlaveOnly", m_peakParams.PtpSlaveOnly);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "PtpSlaveOnly provided was invalid");
    throw ex;
  }

  try {
    declare_parameter("ChunkModeActive", false);
    get_parameter("ChunkModeActive", m_peakParams.ChunkModeActive);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "ChunkModeActive provided was invalid");
    throw ex;
  }

  try {
    declare_parameter("ChunkSelector", "defaultSelector");
    get_parameter("ChunkSelector", m_peakParams.ChunkSelector);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "ChunkSelector provided was invalid");
    throw ex;
  }

  try {
    declare_parameter("ChunkEnable", false);
    get_parameter("ChunkEnable", m_peakParams.ChunkEnable);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "ChunkEnable provided was invalid");
    throw ex;
  }

  getFlashParams();

  RCLCPP_INFO(this->get_logger(), "Setting parameters to:");
  RCLCPP_INFO(this->get_logger(), "  frame_id: %s", m_frameId.c_str());
  RCLCPP_INFO(this->get_logger(), "  image_topic: %s", m_imageTopic.c_str());
  RCLCPP_INFO(this->get_logger(), "  camera_info_url: %s", m_cameraInfoUrl.c_str());
  RCLCPP_INFO(this->get_logger(), "  PixelClock: %i Hz", m_peakParams.PixelClock);
  RCLCPP_INFO(this->get_logger(), "  ExposureTime: %i", m_peakParams.ExposureTime);
  RCLCPP_INFO(this->get_logger(), "  AcquisitionFrameRate: %i", m_peakParams.AcquisitionFrameRate);
  RCLCPP_INFO(this->get_logger(), "  Gamma: %f", m_peakParams.Gamma);
  RCLCPP_INFO(this->get_logger(), "  ImageHeight: %i", m_peakParams.ImageHeight);
  RCLCPP_INFO(this->get_logger(), "  ImageWidth: %i", m_peakParams.ImageWidth);
  RCLCPP_INFO(this->get_logger(), "  OffsetHeight: %i", m_peakParams.OffsetHeight);
  RCLCPP_INFO(this->get_logger(), "  OffsetWidth: %i", m_peakParams.OffsetWidth);
  RCLCPP_INFO(this->get_logger(), "  UseOffset: %i", m_peakParams.UseOffset);
  RCLCPP_INFO(this->get_logger(), "  selectedDevice: %s", m_peakParams.selectedDevice.c_str());
  RCLCPP_INFO(this->get_logger(), "  ExposureAuto: %s", m_peakParams.ExposureAuto.c_str());
  RCLCPP_INFO(this->get_logger(), "  GainAuto: %s", m_peakParams.GainAuto.c_str());
  RCLCPP_INFO(this->get_logger(), "  GainSelector: %s", m_peakParams.GainSelector.c_str());
  RCLCPP_INFO(this->get_logger(), "  PixelFormat: %s", m_peakParams.PixelFormat.c_str());
  RCLCPP_INFO(this->get_logger(), "  TriggerMode: %s", m_peakParams.TriggerMode.c_str());
  RCLCPP_INFO(this->get_logger(), "  TriggerSource: %s", m_peakParams.TriggerSource.c_str());
  RCLCPP_INFO(this->get_logger(), "  TriggerActivation: %s", m_peakParams.TriggerActivation.c_str());
  RCLCPP_INFO(this->get_logger(), "  DeviceLinkThroughputLimit: %i", m_peakParams.DeviceLinkThroughputLimit);
  RCLCPP_INFO(this->get_logger(), "  Line1Source: %s", m_peakParams.Line1Source.c_str());
  RCLCPP_INFO(this->get_logger(), "  TriggerDivider: %i", m_peakParams.TriggerDivider);
  RCLCPP_INFO(this->get_logger(), "  PtpEnable: %i", m_peakParams.PtpEnable);
  RCLCPP_INFO(this->get_logger(), "  PtpSlaveOnly: %i", m_peakParams.PtpSlaveOnly);
  RCLCPP_INFO(this->get_logger(), "  ChunkModeActive: %i", m_peakParams.ChunkModeActive);
  RCLCPP_INFO(this->get_logger(), "  ChunkSelector: %s", m_peakParams.ChunkSelector.c_str());
  RCLCPP_INFO(this->get_logger(), "  ChunkEnable: %i", m_peakParams.ChunkEnable);
  RCLCPP_INFO(this->get_logger(), "  FlashACtive: %i", m_peakParams.FlashActive);
  RCLCPP_INFO(this->get_logger(), "  FlashReference: %s", m_peakParams.FlashReference.c_str());
  RCLCPP_INFO(this->get_logger(), "  FlashDuration: %f", m_peakParams.FlashDuration);
  RCLCPP_INFO(this->get_logger(), "  FlashStartDelay: %f", m_peakParams.FlashStartDelay);
  RCLCPP_INFO(this->get_logger(), "  FlashInvertSignal: %i", m_peakParams.FlashInvertSignal);
}


void PeakCamNode::getFlashParams()
{
  try {
    declare_parameter("FlashActive", false);
    get_parameter("FlashActive", m_peakParams.FlashActive);
  }
  catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "FlashActive provided was invalid");
    throw ex;
  }
  try {
    declare_parameter("FlashReference", "");
    get_parameter("FlashReference", m_peakParams.FlashReference);
  }
  catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "FlashReference provided was invalid");
    throw ex;
  }
  try {
    declare_parameter("FlashDuration", 0.0);
    get_parameter("FlashDuration", m_peakParams.FlashDuration);
  }
  catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "FlashDuration provided was invalid");
    throw ex;
  }
  try {
    declare_parameter("FlashStartDelay", 0.0);
    get_parameter("FlashStartDelay", m_peakParams.FlashStartDelay);
  }
  catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "FlashStartDelay provided was invalid");
    throw ex;
  }
  try {
    declare_parameter("FlashInvertSignal", false);
    get_parameter("FlashInvertSignal", m_peakParams.FlashInvertSignal);
  }
  catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "FlashInvertSignal provided was invalid");
    throw ex;
  }

}

void PeakCamNode::openDevice()
{
  auto& deviceManager = peak::DeviceManager::Instance();  
  //Select Device and set Parameters Once
  while (!m_acquisitionLoopRunning)
  {
    try
    {
      // update the device manager
      deviceManager.Update();
     
      // exit program if no device was found
      if (deviceManager.Devices().empty())
      {
        RCLCPP_INFO(this->get_logger(), "No device found. Exiting program");
        // close library before exiting program
        peak::Library::Close();
        return;
      }
      
      // list all available devices
      size_t i = 0;
      RCLCPP_INFO_ONCE(this->get_logger(), "Devices available: ");
      for (const auto& deviceDescriptor : deviceManager.Devices())
      {
        RCLCPP_INFO(this->get_logger(), "  %lu: %s", i, deviceDescriptor->DisplayName().c_str());
        ++i;
      }
        
      // set i back to 0
      i = 0;
      size_t selectedDevice = 0;
      for (const auto& deviceDescriptor : deviceManager.Devices())
      {
        if (m_peakParams.selectedDevice == deviceDescriptor->SerialNumber())
        {
          RCLCPP_INFO_ONCE(this->get_logger(), "SELECTING NEW DEVICE: %lu", i);
          selectedDevice = i;
        }
        ++i;
      }
   
      // get vector of device descriptors
      auto deviceDesrciptors = deviceManager.Devices();
      // open the selected device
      m_device =
        deviceManager.Devices().at(selectedDevice)->OpenDevice(
          peak::core::DeviceAccessType::Control);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "[PeakCamNode]: " << m_device->ModelName() << " found");
      // get the remote device node map
      m_nodeMapRemoteDevice = m_device->RemoteDevice()->NodeMaps().at(0); 
      std::vector<std::shared_ptr<peak::core::nodes::Node>> nodes = m_nodeMapRemoteDevice->Nodes();
      // sets Acquisition Parameters of the camera -> see yaml
      setDeviceParameters();
      // open the first data stream
      m_dataStream = m_device->DataStreams().at(0)->OpenDataStream(); 
      // get payload size
      auto payloadSize =
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("PayloadSize")->Value();
      
      // get number of buffers to allocate the buffer count depends on your application
      // here the minimum required number for the data stream
      auto bufferCountMax = m_dataStream->NumBuffersAnnouncedMinRequired();
      
      // allocate and announce image buffers and queue them
      for (uint64_t bufferCount = 0; bufferCount < bufferCountMax; ++bufferCount)
      {
          auto buffer = m_dataStream->AllocAndAnnounceBuffer(static_cast<size_t>(payloadSize), nullptr);
          m_dataStream->QueueBuffer(buffer);
      }
      // Lock critical features to prevent them from changing during acqusition
      // m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked")->SetValue(1);

      // start the data stream
      m_dataStream->StartAcquisition();
      // start the device
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>(
        "AcquisitionStart")->Execute();
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>(
        "AcquisitionStart")->WaitUntilDone();
      RCLCPP_INFO_STREAM(
        this->get_logger(), "[PeakCamNode]: " << m_device->ModelName() << " connected");
      m_acquisitionLoopRunning = true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR_STREAM_ONCE(
        this->get_logger(), "[PeakCamNode]: EXCEPTION: " << e.what());
      RCLCPP_ERROR_STREAM_ONCE(
        this->get_logger(),
        "[PeakCamNode]: Device at port " << m_peakParams.selectedDevice <<
        " not connected or must run as root!");
    }
  }
}

void PeakCamNode::setDeviceParameters()
{
  int maxWidth, maxHeight = 0;
  maxWidth = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("WidthMax")->Value();
  // RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: maxWidth '" << maxWidth << "'");
  maxHeight = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("HeightMax")->Value();
  // RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: maxHeight '" << maxHeight << "'");
  // Set Width, Height
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Width")->SetValue(m_peakParams.ImageWidth);
  RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: ImageWidth is set to '" << m_peakParams.ImageWidth << "'");
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Height")->SetValue(m_peakParams.ImageHeight);
  RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: ImageHeight is set to '" << m_peakParams.ImageHeight << "'");
  
  if (m_peakParams.UseOffset) {
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("OffsetX")->SetValue(m_peakParams.OffsetWidth);
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("OffsetY")->SetValue(m_peakParams.OffsetHeight);
  } else {
    // auto-center if UseOffset is set to False
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("OffsetX")->SetValue((maxWidth - m_peakParams.ImageWidth) / 2);
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("OffsetY")->SetValue((maxHeight - m_peakParams.ImageHeight) / 2);
  }

  //Set DeviceLinkThroughputLimit Parameter
  try{
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("DeviceLinkThroughputLimit")->SetValue(m_peakParams.DeviceLinkThroughputLimit);
    RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: DeviceLinkThroughputLimit is set to '" << m_peakParams.DeviceLinkThroughputLimit << "'");
  }catch(const std::exception&)
  {
    RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: DeviceLinkThroughputLimit is not a parameter for this camera ");
  }
  //Set PixelClock (=DeviceClockFrequency[Sensor]) Parameter
  if (setRemoteDeviceParameter(m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("DeviceClockSelector"), "Sensor")) {
    if (setRemoteDeviceParameter(m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("DeviceClockFrequency"), m_peakParams.PixelClock)) {
      RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: PixelClock is set to " << m_peakParams.PixelClock << " Hz");
    }
  }

  //Set GainAuto Parameter
  try{
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainAuto")->SetCurrentEntry(m_peakParams.GainAuto);
    RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: GainAuto is set to '" << m_peakParams.GainAuto << "'");
  }catch(const std::exception&)
  {
    RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: GainAuto is not a parameter for this camera");
  }
  
  //Set GainSelector Parameter
  try{
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")->SetCurrentEntry(m_peakParams.GainSelector);
    RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: GainSelector is set to '" << m_peakParams.GainSelector << "'");
  }catch(const std::exception&)
  {
    RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: GainSelector is not a parameter for this camera");
  }
  
  //Set ExposureAuto Parameter
  try{
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("ExposureAuto")->SetCurrentEntry(m_peakParams.ExposureAuto);
    RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: ExposureAuto is set to '" << m_peakParams.ExposureAuto << "'");
  }catch(const std::exception&)
  {
    RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: ExposureAuto is not a parameter for this camera");
  }
  

  //Set ExposureTime Parameter
  if(m_peakParams.ExposureAuto == "Off")
  {
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ExposureTime")->SetValue(m_peakParams.ExposureTime);
    RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: ExposureTime is set to " << m_peakParams.ExposureTime << " microseconds");
  }
  
  //Set Gamma Parameter
  try{
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gamma")->SetValue(m_peakParams.Gamma);
    RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: Gamma is set to " << m_peakParams.Gamma);
  }catch(const std::exception&)
  {
    RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: Gamma is set not a parameter for this camera");
  }
  
  //Set PixelFormat Parameter
  try{
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat")->SetCurrentEntry(m_peakParams.PixelFormat);
    RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: PixelFormat is set to '" << m_peakParams.PixelFormat << "'");
  }catch(const std::exception&)
  {
    RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: PixelFormat is not a parameter for this camera");
  }
  

  // Set TriggerMode
  if (m_peakParams.TriggerMode == "On" || m_peakParams.TriggerSource.compare("Off") != 0) {
    // TODO(flynneva): add more parameters for customizing trigger
    // trigger acqusition, delayed trigger, etc.
    try{
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("Line1Source")
      ->SetCurrentEntry(m_peakParams.Line1Source);
    }catch(const std::exception&)
    {
    }
    
     m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSelector")
      ->SetCurrentEntry("ExposureStart");
     m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerMode")->SetCurrentEntry("On");
     m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSource")
      ->SetCurrentEntry(m_peakParams.TriggerSource);
     m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerActivation")
      ->SetCurrentEntry(m_peakParams.TriggerActivation);
    try{
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TimerSelector")
      ->SetCurrentEntry("Timer0");
    }catch(const std::exception&)
    {
    }
     
     m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("TriggerDivider")
      ->SetValue(m_peakParams.TriggerDivider);
    try{
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("TimerDuration")->SetValue(500000.0);
    }catch(const std::exception&)
    {
    }
     
    std::string triggerTypeStart = "Timer0";
    // set hardline trigger settings
    std::string lineIn = "Line";
    char *chartrans=new char[m_peakParams.TriggerSource.length() + 1];
    strcpy(chartrans,m_peakParams.TriggerSource.c_str());
    lineIn.push_back(*chartrans);  // GPIO pin number

    // already including some future logic for other trigger types
    try{
      if (triggerTypeStart == "Counter0")
      {
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("CounterSelector")
        ->SetCurrentEntry("Counter0");
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("CounterTriggerSource")
        ->SetCurrentEntry(lineIn);
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("CounterTriggerActivation")
        ->SetCurrentEntry("RisingEdge");
      }
      else if (triggerTypeStart == "Timer0")
      {
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TimerSelector")
        ->SetCurrentEntry("Timer0");
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TimerTriggerSource")
        ->SetCurrentEntry(lineIn);
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TimerTriggerActivation")
        ->SetCurrentEntry("RisingEdge");
      }
      else
      {
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSelector")
        ->SetCurrentEntry(triggerTypeStart);
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSource")
        ->SetCurrentEntry(lineIn);
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerActivation")
        ->SetCurrentEntry("RisingEdge");
      }
    }catch(const std::exception&)
    {
    }
    
  }
  else {
    RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode] No Trigger Specified, running continously");
    try {
      //Set AcquisitionFrameRate Parameter
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")->SetValue(m_peakParams.AcquisitionFrameRate);
      RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: AcquisitionFrameRate is set to " << m_peakParams.AcquisitionFrameRate << " Hz");
    }
    catch(const std::exception&e) {
      // Something failed. Report the failure
      RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: Failed to set AcquisitionFrameRate. Check configuration of the camera. Error was:% "
            << e.what());
    }
  }
    
  // Set Parameters for ROS Image
  try{
    if (m_peakParams.PixelFormat == "Mono8") {
      m_pixelFormat = peak::ipl::PixelFormatName::Mono8;
      m_image_encoding = sensor_msgs::image_encodings::MONO8;
      m_bytesPerPixel = 1;
    } else if (m_peakParams.PixelFormat == "RGB8") {
      m_pixelFormat = peak::ipl::PixelFormatName::RGB8;
      m_image_encoding = sensor_msgs::image_encodings::RGB8;
      m_bytesPerPixel = 1;
    } else if (m_peakParams.PixelFormat == "BGR8") {
      m_pixelFormat = peak::ipl::PixelFormatName::BGR8;
      m_image_encoding = sensor_msgs::image_encodings::BGR8;
      m_bytesPerPixel = 1;
    }
  }catch(const std::exception&)
  {
  }
  // Process PTP related parameters.
  // Not all cameras support PTP, check if yes by trying to read the PtpEnable node, and fail silently
  bool ptpSupported = true;
  try {
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("PtpEnable")->Value();
  }
  catch (const peak::core::NotFoundException &e) {
    // Issue a warning about PTP support only if we are supposed to activate/use PTP
    if (m_peakParams.PtpEnable) {
        RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: PTP does not appear to be supported on this camera: "
                    << e.what()
                    << ". You may ignore this error...");
    }
    ptpSupported = false;
  }
  catch(const std::exception &e) {
    RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: Error accessing parameter PtpEnable: "
                    << e.what());
  }
  if (ptpSupported) {
    if (m_peakParams.PtpEnable) {
      // We are supposed to enable PTP. If it is already enabled, we go on to handle the PtpSlaveOnly parameter.
      // Otherwise we enable and continue to process the PtpSlaveOnly parameter.
      bool enabled = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("PtpEnable")->Value();
      if (! enabled) {
        try {
          m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("PtpEnable")->SetValue(true);
        }
        catch(const std::exception&) {
          RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: PtpEnable is not a writeble parameter for this camera");
        }
      }
      enabled = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("PtpEnable")->Value();
      if (enabled) {
        if (m_peakParams.PtpSlaveOnly) {
          try {
            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("PtpSlaveOnly")->SetValue(m_peakParams.PtpSlaveOnly);
          }
          catch(const std::exception&) {
            RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: PtpSlaveOnly is not a parameter for this camera");
          }
          try {
            int count=0;
            while (m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PtpStatus")->CurrentEntry()->SymbolicValue().compare("Slave") != 0) {
              sleep(3);
              if (count >= 20) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[PeakCamNode]: PTP Slave status expected, got: "
                    << m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PtpStatus")->CurrentEntry()->SymbolicValue());
                break;
              }
              count++;
            }
          }
          catch (const std::exception &e) {
            RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: Failed to query the PTPStatus node of the camera: " << e.what());
          }
        }
        else {
          // Actual status is not relevant, just log it for info
          RCLCPP_INFO_STREAM(get_logger(), "[PeakCamNode]: Current PTP status is "
                    << m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PtpStatus")->CurrentEntry()->SymbolicValue()
                    << ". If not Slave or Master, this may prevent acquisition to start...");
        }
      }
      else {
        // Enabling PTP has failed.
        RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: Failed to enable PTP for this camera");
      }
    }
    else {
      // The PtpEnable parameter is false, but the camera (if PTP is supported) may have its PTP enabled by default, and
      // if its PTP status is not Slave or Master, this may prevent acquisition to start. Force PtpEnable to false as requested by
      // the parameter but fail silently if failure occurs.
      try {
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("PtpEnable")->SetValue(false);
      }
      catch(const std::exception&) {
      }
    }
  }

  try{
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("ChunkModeActive")->SetValue(m_peakParams.ChunkModeActive);
  }catch(const std::exception&)
  {
    RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: ChunkModeActive is not a parameter for this camera");
  }
  try{
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("ChunkSelector")->SetCurrentEntry(m_peakParams.ChunkSelector);
  }catch(const std::exception&)
  {
    RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: ChunkSelector is not a parameter for this camera");
  }
  try{
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("ChunkEnable")->SetValue(m_peakParams.ChunkEnable);
  }catch(const std::exception&)
  {
    RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: ChunkEnable is not a parameter for this camera");
  }
  
  setFlashControlParameters();
  
}


void PeakCamNode::setFlashControlParameters()
{
  if ( ! m_peakParams.FlashActive) {
    // Nothing to do ??
    return;
  }
  if (m_peakParams.FlashDuration == 0) {
    RCLCPP_WARN(this->get_logger(), "[PeakCamNode]: FlashDuration is 0, this will disable Flash, even if FlashActive is True");
  }
  setRemoteDeviceParameter(m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("FlashReference"),
                          m_peakParams.FlashReference);
  if (m_peakParams.FlashReference.compare("ExposureStart") == 0) {
    // Set Flash duration and start delay if the reference is the start of exposure
    setRemoteDeviceParameter(m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("FlashDuration"),
                        m_peakParams.FlashDuration);
    setRemoteDeviceParameter(m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("FlashStartDelay"), m_peakParams.FlashStartDelay);
  }
  // Set the signal polarity based on the FlashInvertSignal parameter. Flash is Line1
  if (setRemoteDeviceParameter(m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("LineSelector"), "Line1")) {
    setRemoteDeviceParameter(m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("LineSource"), "FlashActive");
    setRemoteDeviceParameter(m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("LineInverter"), m_peakParams.FlashInvertSignal);
  }

}


bool PeakCamNode::setRemoteDeviceParameter(std::shared_ptr<peak::core::nodes::EnumerationNode> node, const std::string &value)
{
  try {
    node->SetCurrentEntry(value);
  }
  catch (const peak::core::NotFoundException &e) {
    RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: %s is not a parameter for this camera", node->Name().c_str());
    return false;
  }
  catch (const peak::core::OutOfRangeException &e) {
    RCLCPP_ERROR(this->get_logger(), "[PeakCamNode]: Out of range value for parameter %s: %s", node->Name().c_str(), value.c_str());
    RCLCPP_ERROR(this->get_logger(), "[PeakCamNode]: Permitted values are:");
    for (auto entry : node->Entries()) {
      RCLCPP_INFO(this->get_logger(), "        %s", entry->SymbolicValue().c_str());
    }
    return false;
  }
  catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "[PeakCamNode]: Error accessing " << node->Name() << " parameter: " << e.what());
      return false;
  }
  return true;
}

bool PeakCamNode::setRemoteDeviceParameter(std::shared_ptr<peak::core::nodes::FloatNode> node, const double value)
{
  double actual_value;
  try {
    node->SetValue(value);
    actual_value = node->Value();
  }
  catch (const peak::core::NotFoundException &e) {
    RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: %s is not a parameter for this camera", node->Name().c_str());
    return false;
  }
  catch (const peak::core::OutOfRangeException &e) {
    RCLCPP_ERROR(this->get_logger(), "[PeakCamNode]: Out of range value for parameter %s: %f", node->Name().c_str(), value);
    RCLCPP_ERROR(this->get_logger(), "[PeakCamNode]:    Valid range is [%f, %f]", node->Minimum(), node->Maximum());
    return false;
  }
  catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "[PeakCamNode]: Error accessing " << node->Name() << " parameter: " << e.what());
      return false;
  }
  RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: parameter %s now has value %f", node->Name().c_str(), actual_value);
  return true;
}

bool PeakCamNode::setRemoteDeviceParameter(std::shared_ptr<peak::core::nodes::IntegerNode> node, const int value)
{
  try {
    node->SetValue(value);
  }
  catch (const peak::core::NotFoundException &e) {
    RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: %s is not a parameter for this camera", node->Name().c_str());
    return false;
  }
  catch (const peak::core::OutOfRangeException &e) {
    RCLCPP_ERROR(this->get_logger(), "[PeakCamNode]: Out of range value for parameter %s: %d", node->Name().c_str(), value);
    RCLCPP_ERROR(this->get_logger(), "[PeakCamNode]:    Valid range is [%ld, %ld]", node->Minimum(), node->Maximum());
    return false;
  }
  catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "[PeakCamNode]: Error accessing " << node->Name() << " parameter: " << e.what());
      return false;
  }
  return true;
}

bool PeakCamNode::setRemoteDeviceParameter(std::shared_ptr<peak::core::nodes::BooleanNode> node, const bool value)
{
  try {
    node->SetValue(value);
  }
  catch (const peak::core::NotFoundException &e) {
    RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: %s is not a parameter for this camera", node->Name().c_str());
    return false;
  }
  catch (const peak::core::OutOfRangeException &e) {
    RCLCPP_ERROR(this->get_logger(), "[PeakCamNode]: Out of range value for parameter %s: %d", node->Name().c_str(), value);
    return false;
  }
  catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "[PeakCamNode]: Error accessing " << node->Name() << " parameter: " << e.what());
      return false;
  }
  return true;
}


void PeakCamNode::acquisitionLoop()
{
  while (m_acquisitionLoopRunning) {
    try {
      m_header->stamp = this->now();
      RCLCPP_INFO_ONCE(this->get_logger(), "[PeakCamNode]: Acquisition started");
      // get buffer from data stream and process it
      auto buffer = m_dataStream->WaitForFinishedBuffer(5000);

      if (buffer->HasChunks())
      {
        // update nodemap with current chunk data
        m_nodeMapRemoteDevice->UpdateChunkNodes(buffer);
        m_header->stamp = rclcpp::Time(m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("ChunkTimestamp")->Value());
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(),"time2 reçu cam : "<< m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("ChunkTimestamp")->Value());
      }
      auto ci = m_cameraInfoManager->getCameraInfo();
      m_cameraInfo.reset(new sensor_msgs::msg::CameraInfo(ci));
      m_cameraInfo->header = *m_header;

      const auto imageBufferSize __attribute__((unused)) = m_peakParams.ImageWidth * m_peakParams.ImageHeight * m_bytesPerPixel;
      // buffer processing start
      auto image = peak::BufferTo<peak::ipl::Image>(buffer).ConvertTo(m_pixelFormat);
      cv::Mat cvImage;
      if (m_peakParams.PixelFormat == "Mono8")
        cvImage = cv::Mat::zeros(image.Height(), image.Width(), CV_8UC1);
      else
        cvImage = cv::Mat::zeros(image.Height(), image.Width(), CV_8UC3);
      int sizeBuffer = static_cast<int>(image.ByteCount());
      // Device buffer is being copied into cv_bridge format
      std::memcpy(cvImage.data, image.Data(), static_cast<size_t>(sizeBuffer));
      // cv_bridge Image is converted to sensor_msgs/Image to publish on ROS Topic
      RCLCPP_INFO_ONCE(this->get_logger(), "[PeakCamNode]: cv bridge image");
      m_cvImage.reset(new cv_bridge::CvImage());
      m_cvImage->header = *m_header;
      m_cvImage->encoding = m_image_encoding;
      m_cvImage->image = cvImage;
      m_pubImage->publish(*m_cvImage->toImageMsg());
      m_pubCameraInfo->publish(*m_cameraInfo);
      RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "[PeakCamNode]: Publishing data");
      // queue buffer
      m_dataStream->QueueBuffer(buffer);
    }
    catch (const peak::core::TimeoutException &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[PeakCamNode]: EXCEPTION: " << e.what());
        RCLCPP_ERROR(this->get_logger(), "[PeakCamNode]: Timeout Error: check the triggering of your camera.");
    }
    catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "[PeakCamNode]: EXCEPTION: " << e.what());
      RCLCPP_ERROR(this->get_logger(), "[PeakCamNode]: Acquisition loop stopped, device may be disconnected!");
      RCLCPP_ERROR(this->get_logger(), "[PeakCamNode]: No device reset available");
      RCLCPP_ERROR(this->get_logger(), "[PeakCamNode]: Restart peak cam node!");
    }
  }
}

void PeakCamNode::closeDevice()
{
  // if device was opened, try to stop acquisition
  if (m_device) {
    try {
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop")->Execute();
      RCLCPP_INFO(this->get_logger(), "Executing 'AcquisitionStop'");
      m_acquisitionLoopRunning = false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "EXCEPTION: " << e.what());
    }
  }
  // if data stream was opened, try to stop it and revoke its image buffers
  if (m_dataStream) {
    try {
      m_dataStream->KillWait(); //->KillOneWait();
      m_dataStream->StopAcquisition(peak::core::AcquisitionStopMode::Default);
      m_dataStream->Flush(peak::core::DataStreamFlushMode::DiscardAll);
      for (const auto &buffer : m_dataStream->AnnouncedBuffers()) {
        m_dataStream->RevokeBuffer(buffer);
      }
      RCLCPP_INFO(this->get_logger(), "'AcquisitionStop' Succesful");
      m_acquisitionLoopRunning = false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "EXCEPTION: " << e.what());
    }
  }
}
} // namespace peak_cam

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(peak_cam::PeakCamNode)
