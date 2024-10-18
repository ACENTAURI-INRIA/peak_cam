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
#ifndef PEAK_CAM__ACQUISITION_PARAMETERS_HPP_
#define PEAK_CAM__ACQUISITION_PARAMETERS_HPP_

#include <peak_ipl/peak_ipl.hpp>
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"


namespace peak_cam
{

struct Peak_Params
{
  std::string selectedDevice{"000000"}; // default to all 0's
  int ExposureTime{100};
  int AcquisitionFrameRate{10};
  int ImageHeight{480};
  int ImageWidth{640};
  bool UseOffset{false};
  int OffsetX{0};
  int OffsetY{0};
  bool Binning{false};
  std::string BinningSelector{""};
  int BinningVertical{1};
  int BinningHorizontal{1};
  bool Decimation{false};
  std::string DecimationSelector{"Sensor"};
  std::string DecimationVerticalMode{""};
  int DecimationVertical{1};
  std::string DecimationHorizontalMode{""};
  int DecimationHorizontal{1};
  double Gamma{1.2};
  std::string ExposureAuto{"Off"};
  std::string AutoExposureTimeLimitMode{"Off"};
  double AutoExposureTimeMin{0.0};
  double AutoExposureTimeMax{0.0};
  std::string GainAuto{"Off"};
  std::string GainSelector;
  double Gain{1.0};
  std::string BalanceWhiteAuto{"Off"};
  std::string PixelFormat{"RGB8"};
  std::string ImagePixelFormat{"RGB8"};
  std::string PixelConversionMode{"Fast"};
  std::string TriggerMode{"Off"};
  std::string TriggerSource{"Off"};
  std::string TriggerActivation{"RisingEdge"};
  double PixelClock{0.0};
  int DeviceLinkThroughputLimit{0};
  // std::string Line1Source{"ExposureActive"};
  int TriggerDivider{1};
  bool PtpEnable{false};
  bool PtpSlaveOnly{true};
  bool ChunkModeActive{true};
#ifdef MULTICHUNKS
  std::string EnabledChunks;
  std::string DisabledChunks;
#else
  std::string ChunkSelector{"Timestamp"};
  bool ChunkEnable{true};
#endif
  bool FlashActive{false};
  std::string FlashReference{""};
  double FlashDuration{0.0};
  double FlashStartDelay{1.0};
  bool FlashInvertSignal{false};
};
}
#endif  // PEAK_CAM__ACQUISITION_PARAMETERS_HPP_
