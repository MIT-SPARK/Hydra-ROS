#include "hydra_ros/input/pointcloud_receiver.h"

#include <glog/logging.h>
#include <hydra/common/global_info.h>

#include "hydra_ros/input/pointcloud_adaptor.h"

namespace hydra {

void declare_config(PointcloudReceiver::Config& config) {
  using namespace config;
  name("PointcloudReceiver::Config");
  base<RosDataReceiver::Config>(config);
}

PointcloudReceiver::PointcloudReceiver(const Config& config, size_t sensor_id)
    : RosDataReceiver(config, sensor_id), config(config) {}

bool PointcloudReceiver::initImpl() {
  sub_ = nh_.subscribe(
      "pointcloud", config.queue_size, &PointcloudReceiver::callback, this);
  return true;
}

void PointcloudReceiver::callback(const sensor_msgs::PointCloud2& msg) {
  const auto timestamp_ns = msg.header.stamp.toNSec();
  VLOG(5) << "[Hydra Reconstruction] Got raw pointcloud input @ " << timestamp_ns
          << " [ns]";

  if (!checkInputTimestamp(timestamp_ns)) {
    return;
  }

  auto packet = std::make_shared<CloudInputPacket>(timestamp_ns, sensor_id_);
  fillPointcloudPacket(msg, *packet, false);
  // TODO(nathan) this is brittle, but at least handles kitti
  packet->in_world_frame =
      msg.header.frame_id == GlobalInfo::instance().getFrames().odom;
  queue.push(packet);
}

}  // namespace hydra
