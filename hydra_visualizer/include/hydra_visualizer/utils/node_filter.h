#pragma once
#include <ros/ros.h>
#include <spark_dsg/scene_graph_node.h>
#include <std_msgs/String.h>

namespace hydra {

class NodeFilter {
 public:
  using Ptr = std::shared_ptr<NodeFilter>;
  using Func = std::function<bool(const spark_dsg::SceneGraphNode& node)>;

  struct Config {
    spark_dsg::LayerId layer = spark_dsg::DsgLayers::PLACES;
    spark_dsg::LayerId child_layer = spark_dsg::DsgLayers::OBJECTS;
  } const config;

  NodeFilter(const Config& config, const ros::NodeHandle& nh);

  virtual ~NodeFilter() = default;

  bool hasChange() const { return has_change_; }

  void clearChangeFlag() { has_change_ = false; }

  Func getFilter() const;

 private:
  void handleFilter(const std_msgs::String& msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  bool has_change_;
  std::set<spark_dsg::NodeId> filter_;
};

}  // namespace hydra
