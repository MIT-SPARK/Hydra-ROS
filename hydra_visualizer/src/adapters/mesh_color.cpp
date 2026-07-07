#include "hydra_visualizer/adapters/mesh_color.h"

#include <config_utilities/config.h>
#include <config_utilities/types/eigen_matrix.h>
#include <config_utilities/validation.h>
#include <spark_dsg/colormaps.h>

#include "hydra_visualizer/color/color_parsing.h"

namespace hydra {
namespace {

static const auto splt_reg =
    config::RegistrationWithConfig<MeshColoring,
                                   SplitMeshColoring,
                                   SplitMeshColoring::Config>("SplitMeshColoring");

}

using spark_dsg::Color;
using spark_dsg::Mesh;

void declare_config(UniformMeshColoring::Config& config) {
  using namespace config;
  name("UniformMeshColoring::Config");
  field(config.color, "color");
}

UniformMeshColoring::UniformMeshColoring(const spark_dsg::Color& color)
    : UniformMeshColoring(Config{color}) {}

UniformMeshColoring::UniformMeshColoring(const Config& config) : config(config) {}

void declare_config(SemanticMeshColoring::Config& config) {
  using namespace config;
  name("SemanticMeshColoring::Config");
  field(config.colormap, "colormap");
}

SemanticMeshColoring::SemanticMeshColoring(const Config& config)
    : config(config), colormap_(config.colormap) {}

Color SemanticMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  if (!mesh.has_labels) {
    return Color::black();
  }

  return colormap_.getColor(mesh.label(i));
}

void declare_config(FirstSeenMeshColoring::Config&) {
  config::name("FirstSeenMeshColoring::Config");
}

FirstSeenMeshColoring::FirstSeenMeshColoring() : FirstSeenMeshColoring(Config()) {}

FirstSeenMeshColoring::FirstSeenMeshColoring(const Config&) {}

void FirstSeenMeshColoring::setMesh(const Mesh& mesh) {
  min_ = std::numeric_limits<Mesh::Timestamp>::max();
  max_ = std::numeric_limits<Mesh::Timestamp>::min();
  for (const auto t : mesh.first_seen_stamps) {
    if (t < min_ && t > 0) {
      min_ = t;
    }
    if (t > max_) {
      max_ = t;
    }
  }
}

Color FirstSeenMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  return colorFromTime(mesh.first_seen_stamps[i], min_, max_);
}

void FirstSeenMeshColoring::setBounds(Mesh::Timestamp min, Mesh::Timestamp max) {
  min_ = min;
  max_ = max;
}

void declare_config(LastSeenMeshColoring::Config&) {
  config::name("LastSeenMeshColoring::Config");
}

LastSeenMeshColoring::LastSeenMeshColoring() : LastSeenMeshColoring(Config()) {}

LastSeenMeshColoring::LastSeenMeshColoring(const Config&) {}

void LastSeenMeshColoring::setMesh(const Mesh& mesh) {
  min_ = std::numeric_limits<Mesh::Timestamp>::max();
  max_ = std::numeric_limits<Mesh::Timestamp>::min();
  for (const auto t : mesh.stamps) {
    if (t < min_ && t > 0) {
      min_ = t;
    }
    if (t > max_) {
      max_ = t;
    }
  }
}

Color LastSeenMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  return colorFromTime(mesh.stamps[i], min_, max_);
}

void LastSeenMeshColoring::setBounds(Mesh::Timestamp min, Mesh::Timestamp max) {
  min_ = min;
  max_ = max;
}

void declare_config(SeenDurationMeshColoring::Config&) {
  config::name("SeenDurationMeshColoring::Config");
}

SeenDurationMeshColoring::SeenDurationMeshColoring()
    : SeenDurationMeshColoring(Config()) {}

SeenDurationMeshColoring::SeenDurationMeshColoring(const Config&) {}

void SeenDurationMeshColoring::setMesh(const Mesh& mesh) {
  max_ = 0;
  for (size_t i = 0; i < mesh.stamps.size(); ++i) {
    const auto duration = mesh.stamps[i] - mesh.first_seen_stamps[i];
    if (duration > max_) {
      max_ = duration;
    }
  }
}

Color SeenDurationMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  return colorFromTime(mesh.stamps[i] - mesh.first_seen_stamps[i], 0, max_);
}

void SeenDurationMeshColoring::setMaxDuration(spark_dsg::Mesh::Timestamp max) {
  max_ = max;
}

void declare_config(SplitMeshColoring::Config& config) {
  using namespace config;
  name("SplitMeshColoring::Config");
  config.coloring.setOptional();
  field(config.coloring, "coloring");
  field(config.normal, "normal");
  field(config.origin, "origin");
  field(config.default_color, "default_color");
}

SplitMeshColoring::SplitMeshColoring(const Config& config)
    : config(config::checkValid(config)), coloring_(config.coloring.create()) {}

void SplitMeshColoring::setMesh(const Mesh& mesh) {
  if (coloring_) {
    coloring_->setMesh(mesh);
  }
}

Color SplitMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  const auto& pos = mesh.pos(i);
  const auto dist = config.normal.dot(pos - config.origin);
  if (dist >= 0.0f && coloring_) {
    return coloring_->getVertexColor(mesh, i);
  }

  return mesh.has_colors ? mesh.color(i) : config.default_color;
}

void declare_config(FusionCountMeshColoring::Config&) {
  config::name("FusionCountMeshColoring::Config");
}

FusionCountMeshColoring::FusionCountMeshColoring() : FusionCountMeshColoring(Config()) {}

FusionCountMeshColoring::FusionCountMeshColoring(const Config&) {}

void FusionCountMeshColoring::setMesh(const Mesh& mesh) {
  // Normalize to the max fusion count of the current mesh (per step).
  max_count_ = 1;
  if (!mesh.has_fusion_counts || mesh.fusion_counts.empty()) {
    return;
  }
  for (const auto count : mesh.fusion_counts) {
    if (count > max_count_) {
      max_count_ = count;
    }
  }
}

Color FusionCountMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  if (!mesh.has_fusion_counts || i >= mesh.fusion_counts.size()) {
    return Color::gray();
  }

  // Grey: never fused. Green (1 fusion) -> red (most fused this step).
  const uint32_t count = mesh.fusion_counts[i];
  if (count == 0) {
    return Color::gray();
  }
  const float t =
      max_count_ > 1
          ? std::min(1.0f, static_cast<float>(count - 1) / static_cast<float>(max_count_ - 1))
          : 0.0f;
  return Color(static_cast<uint8_t>(255.0f * t),
               static_cast<uint8_t>(255.0f * (1.0f - t)),
               0);
}

void declare_config(TemporalIslandMeshColoring::Config&) {
  config::name("TemporalIslandMeshColoring::Config");
}

TemporalIslandMeshColoring::TemporalIslandMeshColoring()
    : TemporalIslandMeshColoring(Config()) {}

TemporalIslandMeshColoring::TemporalIslandMeshColoring(const Config&) {}

Color TemporalIslandMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  if (!mesh.has_temporal_island_ids || i >= mesh.temporal_island_ids.size()) {
    return Color::gray();
  }

  const uint32_t island_id = mesh.temporal_island_ids[i];
  if (island_id == 0) {
    return Color::gray();
  }

  // Skip grey-like indices in distinct150 palette (3, 141, 149)
  // These are too similar to the grey background used for non-island vertices
  uint32_t adjusted_id = island_id;
  if (island_id >= 3) adjusted_id++;
  if (island_id >= 140) adjusted_id++;   // accounts for shift, targeting 141
  if (island_id >= 147) adjusted_id++;   // accounts for shift, targeting 149

  return spark_dsg::colormaps::distinct150Id(adjusted_id);
}

// === PrimitiveMembershipMeshColoring ===

void declare_config(PrimitiveMembershipMeshColoring::Config& cfg) {
  using namespace config;
  name("PrimitiveMembershipMeshColoring::Config");
  field(cfg.primitive_voxel_size_m, "primitive_voxel_size_m");
  check(cfg.primitive_voxel_size_m, GT, 0.f, "primitive_voxel_size_m");
}

PrimitiveMembershipMeshColoring::PrimitiveMembershipMeshColoring()
    : PrimitiveMembershipMeshColoring(Config()) {}

PrimitiveMembershipMeshColoring::PrimitiveMembershipMeshColoring(const Config& cfg)
    : config(config::checkValid(cfg)) {}

Color PrimitiveMembershipMeshColoring::getVertexColor(const Mesh& mesh,
                                                     size_t i) const {
  const auto& p = mesh.points[i];
  const float inv = 1.0f / config.primitive_voxel_size_m;
  const int32_t vx = static_cast<int32_t>(std::floor(p.x() * inv));
  const int32_t vy = static_cast<int32_t>(std::floor(p.y() * inv));
  const int32_t vz = static_cast<int32_t>(std::floor(p.z() * inv));
  // 3-axis spatial hash to a 32-bit color seed.
  uint32_t h = 0x9e3779b9u;
  h ^= static_cast<uint32_t>(vx) + 0x9e3779b9u + (h << 6) + (h >> 2);
  h ^= static_cast<uint32_t>(vy) + 0x9e3779b9u + (h << 6) + (h >> 2);
  h ^= static_cast<uint32_t>(vz) + 0x9e3779b9u + (h << 6) + (h >> 2);
  const uint8_t r = (h >> 0) & 0xFF;
  const uint8_t g = (h >> 8) & 0xFF;
  const uint8_t b = (h >> 16) & 0xFF;
  return Color(r, g, b, 255);
}

// === PostFusionVsOnceObservedMeshColoring ===

void declare_config(PostFusionVsOnceObservedMeshColoring::Config&) {
  config::name("PostFusionVsOnceObservedMeshColoring::Config");
}

PostFusionVsOnceObservedMeshColoring::PostFusionVsOnceObservedMeshColoring()
    : PostFusionVsOnceObservedMeshColoring(Config()) {}

PostFusionVsOnceObservedMeshColoring::PostFusionVsOnceObservedMeshColoring(
    const Config&) {}

Color PostFusionVsOnceObservedMeshColoring::getVertexColor(const Mesh& mesh,
                                                          size_t i) const {
  const auto windows = mesh.observationWindowsOf(i);
  return windows.size() > 1 ? Color::green() : Color::red();
}

MeshColorAdapter::MeshColorAdapter(const Mesh& mesh, MeshColoring::ConstPtr coloring)
    : mesh(mesh), coloring_(std::move(coloring)) {
  if (coloring_) {
    const_cast<MeshColoring&>(*coloring_).setMesh(mesh);
    getVertexColor = [this](size_t i) {
      return coloring_->getVertexColor(this->mesh, i);
    };
  } else {
    getVertexColor = [this](size_t i) { return this->mesh.color(i); };
  }
}

Color colorFromTime(Mesh::Timestamp time, Mesh::Timestamp min, Mesh::Timestamp max) {
  static const visualizer::IronbowPalette palette;
  if (time == 0) {
    return Color::green();
  }

  if (time <= min) {
    return palette(0);
  } else if (time >= max) {
    return palette(1);
  }

  return palette(static_cast<double>(time - min) / static_cast<double>(max - min));
}

}  // namespace hydra
