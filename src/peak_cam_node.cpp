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

using rcl_interfaces::msg::ParameterDescriptor;
using peak::core::nodes::EnumerationNode;
using peak::core::nodes::FloatNode;
using peak::core::nodes::BooleanNode;
using peak::core::nodes::IntegerNode;

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
  declareParameter("frame_id", "camera", "Frame used in the ROS2 header of the published images", m_frameId);
  declareParameter("image_topic", "image_raw", "Name of published image topic", m_imageTopic)  ;
  declareParameter("camera_info_url", "defaultCameraInfoUrl", "URL of the camera_info data file, eg. calibration data", m_cameraInfoUrl);
  declareParameter("ExposureTime", 10000, "Exposure time, in  microseconds, of the image sensor", m_peakParams.ExposureTime);
  declareParameter("AcquisitionFrameRate", 30, "Frame rate of the acquisition, in Hz", m_peakParams.AcquisitionFrameRate);
  declareParameter("ImageHeight", 1080, "Image height, in pixels", m_peakParams.ImageHeight);
  declareParameter("ImageWidth", 1920, "Image width, in pixels", m_peakParams.ImageWidth);
  declareParameter("UseOffset", false, "If true, use parameters OffsetHeight and OffsetWidth to specify the image ROI", m_peakParams.UseOffset);
  declareParameter("OffsetHeight", 0, m_peakParams.OffsetHeight);
  declareParameter("OffsetWidth", 0, m_peakParams.OffsetWidth);
  
  declareParameter("Gamma", 1.0, m_peakParams.Gamma);

  declareParameter("selectedDevice", "0000000000", m_peakParams.selectedDevice);

  declareParameter("ExposureAuto", "Off", m_peakParams.ExposureAuto);
  declareParameter("PixelFormat", "Mono8", m_peakParams.PixelFormat);
  
  declareParameter("GainSelector", "All", m_peakParams.GainSelector);
  declareParameter("GainAuto", "Off", m_peakParams.GainAuto);
  declareParameter("Gain", 1.0, m_peakParams.Gain);

  declareParameter("TriggerMode", "Off",
        "Set to \"On\" to activate trigger based acquisition and use other Trigger-related parameters to specify the triggering events, \"Off\" for freerun acquisition using specified frame rate.",
        m_peakParams.TriggerMode);
  declareParameter("TriggerSource", "Software", "", m_peakParams.TriggerSource);
  declareParameter("TriggerActivation", "RisingEdge", "", m_peakParams.TriggerActivation);
  declareParameter("TriggerDivider", 1, "", m_peakParams.TriggerDivider);

  declareParameter("PixelClock", 20.0, "Pixel clock of the image sensor, in MHz", m_peakParams.PixelClock);

  try {
    declare_parameter("DeviceLinkThroughputLimit", 1000000);
    get_parameter("DeviceLinkThroughputLimit", m_peakParams.DeviceLinkThroughputLimit);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "DeviceLinkThroughputLimit provided was invalid");
    throw ex;
  }

  try {
    declare_parameter("Line1Source", "defaultSource");
    get_parameter("Line1Source", m_peakParams.Line1Source);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "Line1Source provided was invalid");
    throw ex;
  }

  declareParameter("PtpEnable", false, "Enable/Disable PTP (IEEE1588) on the camera. Actually used only with camera that support this feature", m_peakParams.PtpEnable);
  declareParameter("PtpSlaveOnly", false, "If PTP is enabled force camera to act as a PTP slave only. Ignored otherwise", m_peakParams.PtpSlaveOnly);

  declareParameter("ChunkModeActive", false, m_peakParams.ChunkModeActive);
  declareParameter("ChunkSelector", "Timestamp", m_peakParams.ChunkSelector);
  declareParameter("ChunkEnable", false, m_peakParams.ChunkEnable);
#if 0
  declareParameter("ChunksEnabled", "", m_peakParams.ChunksEnabled);
#endif
  RCLCPP_INFO(this->get_logger(), "Setting parameters to:");
  RCLCPP_INFO(this->get_logger(), "  frame_id: %s", m_frameId.c_str());
  RCLCPP_INFO(this->get_logger(), "  image_topic: %s", m_imageTopic.c_str());
  RCLCPP_INFO(this->get_logger(), "  camera_info_url: %s", m_cameraInfoUrl.c_str());
  RCLCPP_INFO(this->get_logger(), "  ExposureTime: %i", m_peakParams.ExposureTime);
  RCLCPP_INFO(this->get_logger(), "  AcquisitionFrameRate: %i", m_peakParams.AcquisitionFrameRate);
  RCLCPP_INFO(this->get_logger(), "  Gamma: %f", m_peakParams.Gamma);
  RCLCPP_INFO(this->get_logger(), "  ImageHeight: %i", m_peakParams.ImageHeight);
  RCLCPP_INFO(this->get_logger(), "  ImageWidth: %i", m_peakParams.ImageWidth);
  RCLCPP_INFO(this->get_logger(), "  UseOffset: %i", m_peakParams.UseOffset);
  RCLCPP_INFO(this->get_logger(), "  OffsetHeight: %i", m_peakParams.OffsetHeight);
  RCLCPP_INFO(this->get_logger(), "  OffsetWidth: %i", m_peakParams.OffsetWidth);
  RCLCPP_INFO(this->get_logger(), "  selectedDevice: %s", m_peakParams.selectedDevice.c_str());
  RCLCPP_INFO(this->get_logger(), "  ExposureAuto: %s", m_peakParams.ExposureAuto.c_str());
  RCLCPP_INFO(this->get_logger(), "  GainSelector: %s", m_peakParams.GainSelector.c_str());
  RCLCPP_INFO(this->get_logger(), "  GainAuto: %s", m_peakParams.GainAuto.c_str());
  RCLCPP_INFO(this->get_logger(), "  Gain: %f", m_peakParams.Gain);
  RCLCPP_INFO(this->get_logger(), "  PixelClock: %f MHz", m_peakParams.PixelClock);
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
#if 0
  RCLCPP_INFO(this->get_logger(), "  ChunksEnabled: %s", m_peakParams.ChunksEnabled);
#endif
}

template<typename T>
void PeakCamNode::declareParameter(const std::string &name, const T& default_value, const std::string & description,
        T& storage)
{
  std::ostringstream os;
  os << description << ". Default is " << default_value;
  std::string full_desc = os.str();
  declare_parameter(name, default_value, ParameterDescriptor().set__description(full_desc));
  try {
    get_parameter(name, storage);
  }
  catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "The value provided for parameter '" << name << "' was invalid");
    throw ex;
  }
}

template<typename T>
void PeakCamNode::declareParameter(const std::string &name, const T& default_value, T& storage)
{
  declareParameter(name, default_value, "No description provided", storage);
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

#if 0
std::vector<std::string> split(const std::string& str, const std::string& delim)
{
    std::vector<std::string> result;
    size_t start = 0;

    for (size_t found = str.find(delim); found != std::string::npos; found = str.find(delim, start))
    {
        result.emplace_back(str.begin() + start, str.begin() + found);
        start = found + delim.size();
    }
    if (start != str.size())
        result.emplace_back(str.begin() + start, str.end());
    return result;
}
#endif

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
  }
  //Set PixelClock (=DeviceClockFrequency[Sensor]) Parameter
  if (getRemoteDeviceParameter<EnumerationNode, std::string>("DeviceClockSelector").compare("Sensor") == 0
      || setRemoteDeviceParameter<EnumerationNode>("DeviceClockSelector", std::string("Sensor"))) {
    if (setRemoteDeviceParameter<FloatNode>("DeviceClockFrequency", m_peakParams.PixelClock * 1000 * 1000)) {
      RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: PixelClock is set to " << m_peakParams.PixelClock << " MHz");
    }
    else {
      double f = getRemoteDeviceParameter<FloatNode, double>("DeviceClockFrequency");
      RCLCPP_INFO_STREAM(get_logger(), "[PeakCamNode]: Failed to set PixelClock, current value is " << f/(1000*1000) << " MHz");
    }
  }

  //Set GainSelector Parameter, must be done before GainAuto or Gain
  if (setRemoteDeviceParameter<EnumerationNode>("GainSelector", m_peakParams.GainSelector)) {
    setRemoteDeviceParameter<EnumerationNode>("GainAuto", m_peakParams.GainAuto);
    if (m_peakParams.GainAuto.compare("Off") == 0) {
      if (setRemoteDeviceParameter<FloatNode>("Gain", m_peakParams.Gain)) {
        double gain = getRemoteDeviceParameter<FloatNode, double>("Gain");
        RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: Gain[" << m_peakParams.GainSelector << "] set to " << gain);
      }
    }
  }
  
  //Set Exposure related Parameter
  if(m_peakParams.ExposureAuto == "Off") {
    if (setRemoteDeviceParameter<EnumerationNode>("ExposureAuto", m_peakParams.ExposureAuto)) {
      RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: ExposureAuto is set to '" << m_peakParams.ExposureAuto << "'");
    }
    else {
      RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: ExposureAuto cannot be set in the camera.");
    }
    // Even if ExposureAuto is not a node/parameter, it may be possible to set the
    // ExposureTime, eg. on old Ueye camera.
    if (setRemoteDeviceParameter<EnumerationNode>("ExposureMode", std::string("Timed"))) {
      if (setRemoteDeviceParameter<FloatNode>("ExposureTime", m_peakParams.ExposureTime)) {
        double v = getRemoteDeviceParameter<FloatNode, double>("ExposureTime");
        RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: ExposureTime is set to " << v << " us");
      }
    }
  }
  else {
    setRemoteDeviceParameter<EnumerationNode>("ExposureAuto", m_peakParams.ExposureAuto);
    RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: ExposureAuto is set to '" << m_peakParams.ExposureAuto << "'");
  }

  //Set Gamma Parameter
  if (setRemoteDeviceParameter<FloatNode>("Gamma", m_peakParams.Gamma)) {
    double v = getRemoteDeviceParameter<FloatNode, double>("Gamma");
    RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: Gamma is set to " << v);
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
  if (m_peakParams.TriggerMode == "On") {
    // TODO(flynneva): add more parameters for customizing trigger
    // trigger acqusition, delayed trigger, etc.
    setRemoteDeviceParameter<EnumerationNode>("Line1Source", m_peakParams.Line1Source);

    // Apply all trigger related parameters to the "ExposureStart" trigger
    std::string selected_trigger = "ExposureStart";
    if (setRemoteDeviceParameter<EnumerationNode>("TriggerSelector", selected_trigger)) {
      setRemoteDeviceParameter<EnumerationNode>("TriggerMode", std::string("On"));
      setRemoteDeviceParameter<EnumerationNode>("TriggerActivation", m_peakParams.TriggerActivation);
      setRemoteDeviceParameter<EnumerationNode>("TriggerSource", m_peakParams.TriggerSource);
      setRemoteDeviceParameter<IntegerNode>("TriggerDivider", m_peakParams.TriggerDivider);
    }

    std::string triggerTypeStart = "Timer0";
    setRemoteDeviceParameter<EnumerationNode>("TimerSelector", triggerTypeStart);
    setRemoteDeviceParameter<FloatNode>("TimerDuration", 500000.0);

    // set hardline trigger settings
    std::string lineIn = "Line";
    char *chartrans=new char[m_peakParams.TriggerSource.length() + 1];
    strcpy(chartrans,m_peakParams.TriggerSource.c_str());
    lineIn.push_back(*chartrans);  // GPIO pin number

    // already including some future logic for other trigger types
    // TODO test and experiment with various cameras
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
    RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: No Trigger Specified, running continuously");
    std::string selected_trigger = "ExposureStart";
    if (setRemoteDeviceParameter<EnumerationNode>("TriggerSelector", selected_trigger)) {
      if (setRemoteDeviceParameter<EnumerationNode>("TriggerMode", std::string("Off"))) {
        if (setRemoteDeviceParameter<FloatNode>("AcquisitionFrameRate", m_peakParams.AcquisitionFrameRate)) {
          double v = getRemoteDeviceParameter<FloatNode, double>("AcquisitionFrameRate");
          RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: AcquisitionFrameRate is set to " << v << " Hz");
        }
      }
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
  setPTPDeviceParameters();

  if (setRemoteDeviceParameter<BooleanNode>("ChunkModeActive", m_peakParams.ChunkModeActive)) {
#if 0
    if (m_peakParams.ChunkModeActive && ! m_peakParams.ChunksEnabled.empty()) {
      for (std::string &chunk : split(m_peakParams.ChunksEnabled, ",")) {
        RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: Enabling chunk " << chunk);
        if (setRemoteDeviceParameter<EnumerationNode>("ChunkSelector", chunk)) {
          setRemoteDeviceParameter<BooleanNode>("ChunkEnable", true);
        }
      }
    }
#endif
    if (m_peakParams.ChunkModeActive && ! m_peakParams.ChunkSelector.empty()) {
      if (setRemoteDeviceParameter<EnumerationNode>("ChunkSelector", m_peakParams.ChunkSelector)) {
        setRemoteDeviceParameter<BooleanNode>("ChunkEnable", m_peakParams.ChunkEnable);
      }
    }
  }
}

void PeakCamNode::setPTPDeviceParameters()
{
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
    ptpSupported = false;
  }
  if (ptpSupported) {
    if (m_peakParams.PtpEnable) {
      // We are supposed to enable PTP. If it is already enabled, we go on to handle the PtpSlaveOnly parameter.
      // Otherwise we enable and continue to process the PtpSlaveOnly parameter.
      bool enabled = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("PtpEnable")->Value();
      if (! enabled) {
        setRemoteDeviceParameter<BooleanNode>("PtpEnable", true);
      }
      enabled = getRemoteDeviceParameter<BooleanNode, bool>("PtpEnable");
      if (enabled) {
        if (m_peakParams.PtpSlaveOnly) {
          if (setRemoteDeviceParameter<BooleanNode>("PtpSlaveOnly", true)) {
            try {
              int count=0;
              while (getRemoteDeviceParameter<EnumerationNode, std::string>("PtpStatus").compare("Slave") != 0) {
                // m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PtpStatus")->CurrentEntry()->SymbolicValue().compare("Slave") != 0) {
                sleep(3);
                if (count >= 20) {
                  std::string v = getRemoteDeviceParameter<EnumerationNode, std::string>("PtpStatus");
                  RCLCPP_ERROR_STREAM(this->get_logger(), "[PeakCamNode]: PTP Slave status expected, got: " << v);
                  break;
                }
                count++;
              }
            }
            catch (const std::exception &e) {
              RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: Failed to query the PTPStatus node of the camera: " << e.what());
            }
          }
        }
        else {
          // Actual status is not relevant, just log it for info
          std::string v = getRemoteDeviceParameter<EnumerationNode, std::string>("PtpStatus");
          RCLCPP_INFO_STREAM(get_logger(), "[PeakCamNode]: Current PTP status is "
                      << v << ". If not Slave or Master, this may prevent acquisition to start...");
        }
      }
      else {
          // Enabling PTP has failed.
          RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: Failed to enable PTP for this camera");
      }
    }
    else {
      // The PtpEnable parameter is false, but the camera (if PTP is supported) may have its PTP enabled by default, and
      // if its PTP status is not Slave or Master, this may prevent acquisition to start. Force PtpEnable to false as requested
      // by the parameter.
      setRemoteDeviceParameter<BooleanNode>("PtpEnable", false);
    }
  }

}

template<class N, typename V>
bool PeakCamNode::setRemoteDeviceParameter(const std::string &node_name, const V &value)
{
  std::shared_ptr<N> node;
  try {
    node = m_nodeMapRemoteDevice->FindNode<N>(node_name);
    auto access_status = node->AccessStatus();
    bool readable = (access_status == peak::core::nodes::NodeAccessStatus::ReadOnly || access_status == peak::core::nodes::NodeAccessStatus::ReadWrite);
    bool writable = (access_status == peak::core::nodes::NodeAccessStatus::WriteOnly || access_status == peak::core::nodes::NodeAccessStatus::ReadWrite);
    if (writable) {
      setRemoteNodeValue(node, value);
    }
    else if (readable) {
      V actual_value = getRemoteNodeValue(node);
      return (actual_value == value);
    }
    else {
      return false;
    }
  }
  catch (const peak::core::NotFoundException &e) {
    RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: %s is not a parameter for this camera", node_name.c_str());
    return false;
  }
  catch (const peak::core::NotImplementedException &e) {
    RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: %s is not implemented for this camera", node_name.c_str());
    return false;
  }
  catch (const peak::core::BadAccessException &e) {
    RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: %s is not accessible for this camera", node_name.c_str());
    return false;
  }
  catch (const peak::core::NotAvailableException &e) {
    RCLCPP_INFO(this->get_logger(), "[PeakCamNode]: %s is not available for this camera", node_name.c_str());
    return false;
  }
  catch (const peak::core::OutOfRangeException &e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "[PeakCamNode]: Out of range value for parameter " << node_name << ": " << value);
    describeRemoteNodeRange(node);
    return false;
  }
  catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "[PeakCamNode]: Error accessing " << node_name << " parameter: " << e.what());
    return false;
  }
  return true;
}

void PeakCamNode::describeRemoteNodeRange(std::shared_ptr<EnumerationNode> node)
{
  RCLCPP_ERROR(this->get_logger(), "[PeakCamNode]: Permitted values are:");
  for (auto entry : node->Entries()) {
    RCLCPP_INFO(this->get_logger(), "        %s", entry->SymbolicValue().c_str());
  }
}

void PeakCamNode::describeRemoteNodeRange(std::shared_ptr<FloatNode> node)
{
  RCLCPP_ERROR(this->get_logger(), "[PeakCamNode]:    Valid range is [%f, %f]", node->Minimum(), node->Maximum());
}

void PeakCamNode::describeRemoteNodeRange(std::shared_ptr<IntegerNode> node)
{
  RCLCPP_ERROR(this->get_logger(), "[PeakCamNode]:    Valid range is [%ld, %ld]", node->Minimum(), node->Maximum());
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
