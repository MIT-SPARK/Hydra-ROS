#include "hydra_ros/visualizer/feature_color_adaptors.h"

#include <config_utilities/parsing/ros.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/openset/embedding_distances.h>
#include <hydra/openset/embedding_group.h>
#include <hydra_visualizer/color/color_parsing.h>
#include <spark_dsg/colormaps.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include "hydra_ros_build_config.h"

#if defined(HYDRA_USE_SEMANTIC_INFERENCE) && HYDRA_USE_SEMANTIC_INFERENCE
#include <semantic_inference_msgs/FeatureVectorStamped.h>
#endif

namespace hydra {

using namespace spark_dsg;

namespace {

std::string wrapLabel(const std::string& original, size_t width) {
  if (width == 0) {
    return original;
  }

  std::vector<std::string> words;
  size_t pos = 0;
  while (pos < original.size()) {
    const auto prev_pos = pos;
    pos = original.find(' ', pos);
    const auto count = pos == std::string::npos ? pos : pos - prev_pos;
    words.push_back(original.substr(prev_pos, count));
    if (pos != std::string::npos) {
      ++pos;
    }
  }

  size_t curr_width = 0;
  std::string to_return;
  auto iter = words.begin();
  while (iter != words.end()) {
    to_return += *iter;
    curr_width += iter->size();
    ++iter;
    if (iter != words.end()) {
      if (curr_width >= width) {
        to_return += "\n";
        curr_width = 0;
      } else {
        to_return += " ";
        ++curr_width;
      }
    }
  }

  return to_return;
}

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

}  // namespace

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
  return spark_dsg::colormaps::ironbow(std::clamp(ratio, 0.0, 1.0));
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
  config.metric.setOptional();
  field(config.metric, "metric");
  config.features.setOptional();
  field(config.features, "features");
  field(config.colormap, "colormap");
}

NearestFeatureColor::NearestFeatureColor(const Config& config)
    : config(config::checkValid(config)),
      metric_(config.metric.create()),
      features_(config.features.create()),
      colormap_(config.colormap) {}

Color NearestFeatureColor::getColor(const DynamicSceneGraph&,
                                    const SceneGraphNode& node) const {
  if (!features_ || !metric_) {
    return Color();
  }

  const auto& attrs = node.attributes<SemanticNodeAttributes>();
  const auto result = features_->getBestScore(*metric_, attrs.semantic_feature);
  return colormap_.getColor(result.index);
}

void declare_config(NearestFeatureLabel::Config& config) {
  using namespace config;
  name("NearestFeatureColor::Config");
  config.metric.setOptional();
  field(config.metric, "metric");
  config.features.setOptional();
  field(config.features, "features");
  field(config.label_width, "label_width");
}

NearestFeatureLabel::NearestFeatureLabel(const Config& config)
    : config(config::checkValid(config)),
      metric_(config.metric.create()),
      features_(config.features.create()) {}

std::string NearestFeatureLabel::getLabel(const SceneGraphNode& node) const {
  if (!features_ || !metric_) {
    return "";
  }

  const auto& attrs = node.attributes<SemanticNodeAttributes>();
  const auto result = features_->getBestScore(*metric_, attrs.semantic_feature);
  if (result.index >= features_->names.size()) {
    return "";
  }

  auto name = features_->names[result.index];
  return wrapLabel(name, config.label_width);
}

}  // namespace hydra
