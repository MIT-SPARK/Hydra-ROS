/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra_ros/input/image_receiver.h"

#include <config_utilities/config.h>
#include <glog/logging.h>

#include <cv_bridge/cv_bridge.hpp>

namespace hydra {

using semantic_inference_msgs::msg::FeatureImage;
using sensor_msgs::msg::Image;

ColorSubscriber::ColorSubscriber() = default;

ColorSubscriber::ColorSubscriber(ianvs::NodeHandle nh, uint32_t queue_size)
    : impl_(std::make_shared<FilterSub<Image>>(nh, "rgb/image_raw", queue_size)) {}

ColorSubscriber::~ColorSubscriber() = default;

ColorSubscriber::Filter& ColorSubscriber::getFilter() const {
  return *CHECK_NOTNULL(impl_);
}

void ColorSubscriber::fillInput(const Image& img, ImageInputPacket& packet) const {
  try {
    packet.color = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8)->image;
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "Failed to convert color image: " << e.what();
  }
}

DepthSubscriber::DepthSubscriber() = default;

DepthSubscriber::DepthSubscriber(ianvs::NodeHandle nh, uint32_t queue_size)
    : impl_(std::make_shared<FilterSub<Image>>(
          nh, "depth_registered/image_rect", queue_size)) {}

DepthSubscriber::~DepthSubscriber() = default;

DepthSubscriber::Filter& DepthSubscriber::getFilter() const {
  return *CHECK_NOTNULL(impl_);
}

void DepthSubscriber::fillInput(const Image& img, ImageInputPacket& packet) const {
  try {
    packet.depth = cv_bridge::toCvCopy(img)->image;
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "Failed to convert depth image: " << e.what();
  }
}

LabelSubscriber::LabelSubscriber() = default;

LabelSubscriber::LabelSubscriber(ianvs::NodeHandle nh, uint32_t queue_size)
    : impl_(std::make_shared<FilterSub<Image>>(nh, "semantic/image_raw", queue_size)) {}

LabelSubscriber::~LabelSubscriber() = default;

LabelSubscriber::Filter& LabelSubscriber::getFilter() const {
  return *CHECK_NOTNULL(impl_);
}

void LabelSubscriber::fillInput(const Image& img, ImageInputPacket& packet) const {
  try {
    packet.labels = cv_bridge::toCvCopy(img)->image;
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "Failed to convert label image: " << e.what();
  }
}

FeatureSubscriber::FeatureSubscriber() = default;

FeatureSubscriber::FeatureSubscriber(ianvs::NodeHandle nh, uint32_t queue_size)
    : impl_(std::make_shared<FilterSub<FeatureImage>>(
          nh, "semantic/image_raw", queue_size)) {}

FeatureSubscriber::~FeatureSubscriber() = default;

FeatureSubscriber::Filter& FeatureSubscriber::getFilter() const {
  return *CHECK_NOTNULL(impl_);
}

void FeatureSubscriber::fillInput(const MsgType& msg, ImageInputPacket& packet) const {
  try {
    packet.labels = cv_bridge::toCvCopy(msg.image)->image;
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "Failed to convert depth image: " << e.what();
  }

  CHECK_EQ(msg.mask_ids.size(), msg.features.size());
  for (size_t i = 0; i < msg.mask_ids.size(); ++i) {
    const auto& vec = msg.features[i].data;
    packet.label_features.emplace(
        msg.mask_ids[i],
        Eigen::Map<const hydra::FeatureVector>(vec.data(), vec.size()));
  }
}

void declare_config(ClosedSetImageReceiver::Config& config) {
  using namespace config;
  name("ClosedSetImageReceiver::Config");
  base<RosDataReceiver::Config>(config);
}

ClosedSetImageReceiver::ClosedSetImageReceiver(const Config& config,
                                               const std::string& sensor_name)
    : ImageReceiverImpl<LabelSubscriber>(config, sensor_name) {}

void declare_config(OpenSetImageReceiver::Config& config) {
  using namespace config;
  name("OpenSetImageReceiver::Config");
  base<hydra::RosDataReceiver::Config>(config);
}

OpenSetImageReceiver::OpenSetImageReceiver(const Config& config,
                                           const std::string& sensor_name)
    : ImageReceiverImpl<FeatureSubscriber>(config, sensor_name) {}

namespace {

static const auto closed_registration =
    config::RegistrationWithConfig<DataReceiver,
                                   ClosedSetImageReceiver,
                                   ClosedSetImageReceiver::Config,
                                   std::string>("ClosedSetImageReceiver");

static const auto open_registration =
    config::RegistrationWithConfig<hydra::DataReceiver,
                                   OpenSetImageReceiver,
                                   OpenSetImageReceiver::Config,
                                   std::string>("OpenSetImageReceiver");

}  // namespace

}  // namespace hydra
