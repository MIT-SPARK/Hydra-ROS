#include "hydra_ros/visualizer/feature_color_adaptors.h"

#include <config_utilities/parsing/ros.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/config_utilities.h>
#include <hydra/openset/embedding_distances.h>
#include <hydra/openset/embedding_group.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include "hydra_ros_build_config.h"

#if defined(HYDRA_USE_SEMANTIC_INFERENCE) && HYDRA_USE_SEMANTIC_INFERENCE
#include <semantic_inference_msgs/FeatureVectorStamped.h>
#endif

namespace hydra {

using namespace spark_dsg;

std::string showVec(const Eigen::MatrixXf& vec, size_t max_length = 100) {
  if (vec.rows() * vec.cols() == 0) {
    return "[]";
  }

  std::stringstream ss;
  ss << "[";
  for (int i = 0; i < vec.rows(); ++i) {
    ss << std::setprecision(3) << vec(i, 0);
    if (i < vec.rows() - 1) {
      ss << ", ";
    }

    if (ss.str().size() >= max_length) {
      ss << "...";
      break;
    }
  }
  ss << "]";

  return ss.str();
}

void declare_config(FeatureScoreColor::Config& config) {
  using namespace config;
  name("FeatureScoreColor::Config");
  field(config.ns, "ns");
  field(config.min_score, "min_score");
  field(config.max_score, "max_score");
  field(config.use_fixed_range, "use_fixed_range");
  field(config.metric, "metric");
}

#if defined(HYDRA_USE_SEMANTIC_INFERENCE) && HYDRA_USE_SEMANTIC_INFERENCE
using semantic_inference_msgs::FeatureVectorStamped;

struct FeatureScoreColor::FeatureTrampoline {
  void callback(const FeatureVectorStamped& msg) {
    const auto& vec = msg.feature.data;
    trampoline(Eigen::Map<const Eigen::VectorXf>(vec.data(), vec.size()));
  }

  std::function<void(const Eigen::VectorXf&)> trampoline;
};
#else
struct FeatureScoreColor::FeatureTrampoline {};
#endif

FeatureScoreColor::FeatureScoreColor(const Config& config)
    : config(config),
      nh_(config.ns),
      has_change_(false),
      has_feature_(false),
      metric_(config.metric.create()) {
#if defined(HYDRA_USE_SEMANTIC_INFERENCE) && HYDRA_USE_SEMANTIC_INFERENCE
  trampoline_.reset(new FeatureTrampoline());
  trampoline_->trampoline = [this](const auto& vec) { setFeature(vec); };
  sub_ = nh_.subscribe("feature", 1, &FeatureTrampoline::callback, trampoline_.get());
#else
  LOG(ERROR) << "semantic_inference_msgs not found when building, disabled!";
#endif
}

FeatureScoreColor::~FeatureScoreColor() = default;

void FeatureScoreColor::setGraph(const DynamicSceneGraph& graph, LayerId layer_id) {
  values_.clear();
  if (!has_feature_ || !metric_) {
    return;
  }

  const auto& layer = graph.getLayer(layer_id);
  range_.min = 1.0f;
  range_.max = 0.0f;
  for (const auto& [node_id, node] : layer.nodes()) {
    const auto& attrs = node->attributes<SemanticNodeAttributes>();
    const auto score = metric_->score(feature_, attrs.semantic_feature);
    VLOG(20) << "node " << NodeSymbol(node_id).getLabel() << " -> " << score << ": "
             << showVec(attrs.semantic_feature);
    range_.min = std::min(range_.min, score);
    range_.max = std::max(range_.max, score);
    values_[node_id] = score;
  }

  if (config.use_fixed_range) {
    range_ = {config.min_score, config.max_score};
  }

  VLOG(2) << "score range: [" << range_.min << ", " << range_.max << "]";
}

Color FeatureScoreColor::getColor(const DynamicSceneGraph&,
                                  const SceneGraphNode& node) const {
  auto iter = values_.find(node.id);
  if (iter == values_.end()) {
    return Color();
  }

  double ratio = (iter->second - range_.min) / (range_.max - range_.min);
  return Color::ironbow(std::clamp(ratio, 0.0, 1.0));
}

void FeatureScoreColor::setFeature(const Eigen::VectorXf& feature) {
  VLOG(1) << "Got new task!";
  feature_ = feature;
  has_feature_ = true;
  has_change_ = true;
}

void declare_config(NearestFeatureColor::Config& config) {
  using namespace config;
  name("NearestFeatureColor::Config");
  field(config.metric, "metric");
  field(config.features, "features");
}

NearestFeatureColor::NearestFeatureColor(const Config& config) : config(config) {}

Color NearestFeatureColor::getColor(const DynamicSceneGraph&,
                                    const SceneGraphNode& node) const {
  if (!features_ || !metric_) {
    return Color();
  }

  const auto& attrs = node.attributes<SemanticNodeAttributes>();
  const auto result = features_->getBestScore(*metric_, attrs.semantic_feature);
  const auto ratio = static_cast<double>(result.index) / features_->size();
  return Color::rainbow(std::clamp(ratio, 0.0, 1.0));
}

}  // namespace hydra
