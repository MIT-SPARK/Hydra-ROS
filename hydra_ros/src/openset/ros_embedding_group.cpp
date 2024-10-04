#include "hydra_ros/openset/ros_embedding_group.h"

#include <config_utilities/config.h>
#include <glog/logging.h>
#include <hydra/openset/openset_types.h>
#include <ros/ros.h>

#include "hydra_ros_build_config.h"

#if defined(HYDRA_USE_SEMANTIC_INFERENCE) && HYDRA_USE_SEMANTIC_INFERENCE
#include <semantic_inference_msgs/EncodeFeature.h>
#include <semantic_inference_msgs/FeatureVectors.h>
#endif

namespace hydra {

template <typename T>
struct MessageWaitFunctor {
  void callback(const T& msg) { msg_ = msg; }

  std::optional<T> wait() {
    ros::WallRate r(10.0);
    while (ros::ok() && !msg_) {
      r.sleep();
      ros::spinOnce();
    }

    return msg_;
  }

 private:
  std::optional<T> msg_;
};

void declare_config(RosEmbeddingGroup::Config& config) {
  using namespace config;
  name("RosEmbeddingGroup::Config");
  field(config.ns, "ns");
  field(config.silent_wait, "silent_wait");
  field(config.prompts, "prompts");
}

RosEmbeddingGroup::RosEmbeddingGroup(const Config& config) {
#if defined(HYDRA_USE_SEMANTIC_INFERENCE) && HYDRA_USE_SEMANTIC_INFERENCE
  if (config.prompts.empty()) {
    ros::NodeHandle nh(config.ns);
    LOG_IF(INFO, !config.silent_wait)
        << "Waiting for embeddings on '" << nh.resolveName("features") << "'";
    MessageWaitFunctor<semantic_inference_msgs::FeatureVectors> functor;
    auto sub = nh.subscribe(
        "features",
        1,
        &MessageWaitFunctor<semantic_inference_msgs::FeatureVectors>::callback,
        &functor);
    const auto msg = functor.wait();
    if (!msg) {
      LOG(ERROR) << "Failed to get embeddings from '" << nh.resolveName("features")
                 << "'";
      return;
    }

    for (size_t i = 0; i < msg->features.size(); ++i) {
      const auto& vec = msg->features[i].data;
      embeddings.emplace_back(Eigen::Map<const FeatureVector>(vec.data(), vec.size()));
      if (i < msg->names.size()) {
        names.push_back(msg->names[i]);
      }
    }

    LOG_IF(INFO, !config.silent_wait)
        << "Got embeddings from '" << nh.resolveName("features") << "'!";
    return;
  }

  const std::string service_name = ros::names::append(config.ns, "embed");
  LOG_IF(INFO, !config.silent_wait) << "Waiting for embedding encoder on '"
                                    << ros::names::resolve(service_name) << "'...";
  ros::service::waitForService(service_name);

  for (const auto& prompt : config.prompts) {
    ::semantic_inference_msgs::EncodeFeature msg;
    msg.request.prompt = prompt;
    CHECK(ros::service::call(service_name, msg));
    names.push_back(prompt);
    const auto& vec = msg.response.feature.feature.data;
    embeddings.emplace_back(Eigen::Map<const FeatureVector>(vec.data(), vec.size()));
  }

  LOG_IF(INFO, !config.silent_wait) << "Finished embedding prompts on using '"
                                    << ros::names::resolve(service_name) << "'!";
#else
  LOG(ERROR) << "Hydra not built with semantic_inference. Not encoding prompts";
#endif
}

}  // namespace hydra
