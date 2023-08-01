#include "hydra_ros/utils/image_receiver.h"

#include <glog/logging.h>

#include "hydra_ros/utils/image_to_pointcloud.h"

namespace hydra {

ImageSubscriber::ImageSubscriber(const ros::NodeHandle& nh,
                                 const std::string& camera_name,
                                 const std::string& image_name,
                                 uint32_t queue_size)
    : transport(ros::NodeHandle(nh, camera_name)),
      sub(transport,
          image_name,
          queue_size,
          image_transport::TransportHints(
              "raw", ros::TransportHints(), ros::NodeHandle(nh, camera_name))) {}

ImageReceiver::ImageReceiver(const ros::NodeHandle& nh,
                             const OutputCallback& callback,
                             size_t queue_size)
    : nh_(nh),
      callback_(callback),
      color_sub_(nh_, "rgb"),
      depth_sub_(nh_, "depth_registered", "image_rect"),
      label_sub_(nh_, "semantic") {
  info_sub_.subscribe(nh_, "rgb/camera_info", 1);

  synchronizer_.reset(new Synchronizer(SyncPolicy(queue_size),
                                       color_sub_.sub,
                                       depth_sub_.sub,
                                       label_sub_.sub,
                                       info_sub_));
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
                             const sensor_msgs::Image::ConstPtr& labels,
                             const sensor_msgs::CameraInfo::ConstPtr& info) {
  if (color->width != depth->width || color->height != depth->height) {
    LOG(ERROR) << "color dimensions do not match depth dimensions: "
               << showImageDim(color) << " != " << showImageDim(depth);
    return;
  }

  if (labels->width != depth->width || labels->height != depth->height) {
    LOG(ERROR) << "label dimensions do not match depth dimensions: "
               << showImageDim(labels) << " != " << showImageDim(depth);
    return;
  }

  // TODO(nathan) check encodings

  sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2());
  cloud->header = color->header;
  fillPointcloudFromImages(*color, *depth, *labels, *info, *cloud);
  callback_(cloud);
}

}  // namespace hydra
