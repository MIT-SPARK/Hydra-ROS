#include "hydra_ros/backend/gt_room_publisher.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hydra {

void declare_config(GtRoomPublisher::Config& config) {
  using namespace config;
  name("GtRoomPublisher::Config");
  field(config.ns, "ns");
}

GtRoomPublisher::GtRoomPublisher(const Config& config)
    : config(config), nh_(ianvs::NodeHandle::this_node(config.ns)) {
  room_publisher_ = nh_.create_publisher<visualization_msgs::msg::MarkerArray>(
      "gt_room_boundaries", 1);
}

std::string GtRoomPublisher::printInfo() const { return config::toString(config); }

void GtRoomPublisher::call(uint64_t, const RoomFinder& rf) const {
  LOG(WARNING) << "GT Room sink called";
  visualization_msgs::msg::MarkerArray ma;
  visualization_msgs::msg::Marker m;
  m.action = m.DELETEALL;
  ma.markers.push_back(m);
  int idx = 0;
  int room_idx = 0;

  const std::vector<double> reds{0, .2, .4, .6, .8, 1};
  const std::vector<double> greens{1, .8, .6, .4, .2};
  const std::vector<double> blues{.4, .2, 0, 1};

  for (auto room : rf.room_extents_.room_bounding_boxes) {
    for (auto box : room) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.ns = "gt_rooms";
      m.id = idx++;
      m.action = m.ADD;
      m.type = m.CUBE;
      m.pose.orientation.w = 1;
      m.pose.position.x = box.world_P_center.x();
      m.pose.position.y = box.world_P_center.y();
      m.pose.position.z = box.world_P_center.z();
      m.scale.x = box.dimensions.x();
      m.scale.y = box.dimensions.y();
      m.scale.z = box.dimensions.z();
      m.color.a = 0.5;
      m.color.r = reds.at(room_idx % reds.size());
      m.color.g = greens.at(room_idx % greens.size());
      m.color.b = blues.at(room_idx % blues.size());
      ma.markers.push_back(m);
    }
    ++room_idx;
  }

  room_publisher_->publish(ma);
}

}  // namespace hydra
