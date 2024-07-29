#include "hydra_ros/openset/ros_embedding_group.h"

#include <config_utilities/config.h>
#include <glog/logging.h>
#include <ros/ros.h>

namespace hydra {

void declare_config(RosEmbeddingGroup::Config& config) {
  using namespace config;
  name("RosEmbeddingGroup::Config");
  field(config.service_ns, "service_ns");
  field(config.silent_wait, "silent_wait");
  field(config.prompts, "prompts");
}

RosEmbeddingGroup::RosEmbeddingGroup(const Config& config) {
  const std::string base_name =
      config.prompts.empty() ? "/get_group" : "/get_embedding";
  const std::string service_name = ros::names::append(config.service_ns, base_name);

  LOG_IF(INFO, !config.silent_wait)
      << "Waiting for task service on '" << ros::names::resolve(service_name) << "'";
  ros::service::waitForService(service_name);

  /*
  if (config.prompts.empty()) {
    ::semantic_inference_msgs::RequestEmbeddingGroup srv;
    CHECK(ros::service::call(service_name, srv));
    const auto& resp = srv.response;
    VLOG(5) << "Got " << resp.names.size() << " features:";
    for (size_t i = 0; i < resp.embeddings.size(); ++i) {
      VLOG(5) << " - " << i << ": " << resp.names[i];
      names.push_back(resp.names[i]);
      const auto& vec = resp.embeddings[i].elements;
      embeddings.emplace_back(
          Eigen::Map<const Eigen::VectorXd>(vec.data(), vec.size()));
    }
  } else {
    for (const auto& prompt : config.prompts) {
      ::semantic_inference_msgs::RequestEmbedding msg;
      msg.request.prompt = prompt;
      CHECK(ros::service::call(service_name, msg));
      names.push_back(prompt);
      const auto& vec = msg.response.embedding.elements;
      embeddings.emplace_back(
          Eigen::Map<const Eigen::VectorXd>(vec.data(), vec.size()));
    }
  }
  */
}

}  // namespace hydra
