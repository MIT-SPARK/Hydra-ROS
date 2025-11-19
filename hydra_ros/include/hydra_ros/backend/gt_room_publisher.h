#pragma once

#include <hydra/backend/update_rooms_functor.h>
#include <ianvs/node_handle.h>

#include <memory>

#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hydra {

class GtRoomPublisher : public UpdateRoomsFunctor::Sink {
 public:
  struct Config {
    std::string ns = "~gt_rooms";
  } const config;

  explicit GtRoomPublisher(const Config&);

  virtual ~GtRoomPublisher() = default;

  std::string printInfo() const;

  void call(uint64_t timestamp_ns, const RoomFinder&) const;

 private:
  ianvs::NodeHandle nh_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr room_publisher_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<UpdateRoomsFunctor::Sink, GtRoomPublisher, Config>(
          "GtRoomPublisher");
};

void declare_config(GtRoomPublisher::Config& config);

}  // namespace hydra
