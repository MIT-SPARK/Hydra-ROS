#pragma once
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

namespace hydra {

struct ImageSubscriber {
  ImageSubscriber(const ros::NodeHandle& nh,
                  const std::string& camera_name,
                  const std::string& image_name = "image_raw",
                  uint32_t queue_size = 1);

  image_transport::ImageTransport transport;
  image_transport::SubscriberFilter sub;
};

class ImageReceiver {
 public:
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                      sensor_msgs::Image,
                                                      sensor_msgs::Image,
                                                      sensor_msgs::CameraInfo>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
  using OutputCallback = std::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>;

  ImageReceiver(const ros::NodeHandle& nh,
                const OutputCallback& callback,
                size_t queue_size = 10);

  virtual ~ImageReceiver();

 private:
  void callback(const sensor_msgs::Image::ConstPtr& color,
                const sensor_msgs::Image::ConstPtr& depth,
                const sensor_msgs::Image::ConstPtr& labels,
                const sensor_msgs::CameraInfo::ConstPtr& info);

  ros::NodeHandle nh_;
  OutputCallback callback_;

  ImageSubscriber color_sub_;
  ImageSubscriber depth_sub_;
  ImageSubscriber label_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;

  std::unique_ptr<Synchronizer> synchronizer_;
};

}  // namespace hydra
