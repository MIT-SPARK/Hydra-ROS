#include "hydra_ros/reconstruction/pointcloud_receiver.h"

#include <glog/logging.h>
#include <hydra/common/hydra_config.h>

#include "hydra_ros/reconstruction/pointcloud_adaptor.h"

namespace hydra {

PointcloudReceiver::PointcloudReceiver(const ros::NodeHandle& nh,
                                       const DataQueue::Ptr& data_queue,
                                       double input_separation_s,
                                       size_t queue_size)
    : DataReceiver(nh, data_queue, input_separation_s) {
  cloud_sub_ =
      nh_.subscribe("pointcloud", queue_size, &PointcloudReceiver::callback, this);
}

PointcloudReceiver::~PointcloudReceiver() {}

void PointcloudReceiver::callback(const sensor_msgs::PointCloud2& msg) {
  VLOG(5) << "[Hydra Reconstruction] Got raw pointcloud input @ "
          << msg.header.stamp.toNSec() << " [ns]";

  if (!checkInputTimestamp(msg.header.stamp)) {
    return;
  }

  auto packet = std::make_shared<CloudInputPacket>(msg.header.stamp.toNSec());
  fillPointcloudPacket(msg, *packet, false);
  // TODO(nathan) this is brittle, but at least handles kitti
  packet->in_world_frame =
      msg.header.frame_id == HydraConfig::instance().getFrames().odom;
  data_queue_->push(packet);
}

}  // namespace hydra
