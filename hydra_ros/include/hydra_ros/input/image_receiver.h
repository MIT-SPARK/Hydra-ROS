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
#pragma once
#include <config_utilities/factory.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include "hydra_ros/input/ros_data_receiver.h"

namespace hydra {

struct ImageSubscriber {
  ImageSubscriber();

  ImageSubscriber(const ros::NodeHandle& nh,
                  const std::string& camera_name,
                  const std::string& image_name = "image_raw",
                  uint32_t queue_size = 1);

  std::shared_ptr<image_transport::ImageTransport> transport;
  std::shared_ptr<image_transport::SubscriberFilter> sub;
};

class ImageReceiver : public RosDataReceiver {
 public:
  using SyncPolicy = message_filters::sync_policies::
      ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  struct Config : RosDataReceiver::Config {
  } const config;

  ImageReceiver(const Config& config, const std::string& sensor_name);
  virtual ~ImageReceiver() = default;

 protected:
  bool initImpl() override;

 private:
  void callback(const sensor_msgs::Image::ConstPtr& color,
                const sensor_msgs::Image::ConstPtr& depth,
                const sensor_msgs::Image::ConstPtr& labels);

  ImageSubscriber color_sub_;
  ImageSubscriber depth_sub_;
  ImageSubscriber label_sub_;
  std::unique_ptr<Synchronizer> synchronizer_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<DataReceiver,
                                     ImageReceiver,
                                     ImageReceiver::Config,
                                     std::string>("ImageReceiver");
};

void declare_config(ImageReceiver::Config& config);

}  // namespace hydra
