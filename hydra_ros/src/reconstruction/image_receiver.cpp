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
#include "hydra_ros/reconstruction/image_receiver.h"

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>

namespace hydra {

image_transport::TransportHints getHintsWithNamespace(const ros::NodeHandle& nh,
                                                      const std::string& ns) {
  return image_transport::TransportHints(
      "raw", ros::TransportHints(), ros::NodeHandle(nh, ns));
}

ImageSubscriber::ImageSubscriber(const ros::NodeHandle& nh,
                                 const std::string& camera_name,
                                 const std::string& image_name,
                                 uint32_t queue_size)
    : transport(ros::NodeHandle(nh, camera_name)),
      sub(transport, image_name, queue_size, getHintsWithNamespace(nh, camera_name)) {}

ImageReceiver::ImageReceiver(const ros::NodeHandle& nh,
                             const DataQueue::Ptr& data_queue,
                             double input_separation_s,
                             size_t queue_size)
    : DataReceiver(nh, data_queue, input_separation_s),
      color_sub_(nh_, "rgb"),
      depth_sub_(nh_, "depth_registered", "image_rect"),
      label_sub_(nh_, "semantic") {
  // TODO(nathan) subscribe to image subsets
  synchronizer_.reset(new Synchronizer(
      SyncPolicy(queue_size), color_sub_.sub, depth_sub_.sub, label_sub_.sub));
  synchronizer_->registerCallback(&ImageReceiver::callback, this);
}

ImageReceiver::~ImageReceiver() {}

std::string showImageDim(const sensor_msgs::Image::ConstPtr& image) {
  std::stringstream ss;
  ss << "[" << image->width << ", " << image->height << "]";
  return ss.str();
}

void ImageReceiver::callback(const sensor_msgs::Image::ConstPtr& color,
                             const sensor_msgs::Image::ConstPtr& depth,
                             const sensor_msgs::Image::ConstPtr& labels) {
  if (color && (color->width != depth->width || color->height != depth->height)) {
    LOG(ERROR) << "color dimensions do not match depth dimensions: "
               << showImageDim(color) << " != " << showImageDim(depth);
    return;
  }

  if (labels && (labels->width != depth->width || labels->height != depth->height)) {
    LOG(ERROR) << "label dimensions do not match depth dimensions: "
               << showImageDim(labels) << " != " << showImageDim(depth);
    return;
  }

  if (!checkInputTimestamp(depth->header.stamp)) {
    return;
  }

  auto packet = std::make_shared<ImageInputPacket>(color->header.stamp.toNSec());
  try {
    const auto cv_depth = cv_bridge::toCvShare(depth);
    packet->depth = cv_depth->image.clone();
    if (color && color->encoding == sensor_msgs::image_encodings::RGB8) {
      auto cv_color = cv_bridge::toCvShare(color);
      packet->color = cv_color->image.clone();
    } else if (color) {
      auto cv_color = cv_bridge::toCvCopy(color, sensor_msgs::image_encodings::RGB8);
      packet->color = cv_color->image;
    }

    if (labels) {
      auto cv_labels = cv_bridge::toCvShare(labels);
      packet->labels = cv_labels->image.clone();
    }
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "unable to read images from ros: " << e.what();
  }

  data_queue_->push(packet);
}

// TODO(nathan) publish input as pointcloud
// if (config_.publish_pointcloud && config_.use_image_receiver) {
// pcl_pub_.publish(cloud);
//}
/*    if (config_.publish_pointcloud) {*/
/*pcl_pub_ = nh_.advertise<PointCloud2>("pointcloud", 3);*/
/*}*/

}  // namespace hydra