#include "hydra_ros/visualizer/mesh_color_adaptor.h"

#include <hydra/common/semantic_color_map.h>
#include <hydra/utils/pgmo_mesh_traits.h>

namespace hydra {

using spark_dsg::Color;
using spark_dsg::Mesh;

FirstSeenMeshColoring::FirstSeenMeshColoring(const Mesh& mesh) {
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

FirstSeenMeshColoring::FirstSeenMeshColoring(Mesh::Timestamp min, Mesh::Timestamp max)
    : min_(min), max_(max) {}

Color FirstSeenMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  return colorFromTime(mesh.first_seen_stamps[i], min_, max_);
}

LastSeenMeshColoring::LastSeenMeshColoring(Mesh::Timestamp min, Mesh::Timestamp max)
    : min_(min), max_(max) {}

LastSeenMeshColoring::LastSeenMeshColoring(const Mesh& mesh) {
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

SeenDurationMeshColoring::SeenDurationMeshColoring(Mesh::Timestamp max) : max_(max) {}

SeenDurationMeshColoring::SeenDurationMeshColoring(const Mesh& mesh) {
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

SemanticMeshColoring::SemanticMeshColoring(const SemanticColorMap& colormap)
    : colormap_(colormap) {}

Color SemanticMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  if (!mesh.has_labels) {
    return Color::black();
  }
  const auto label = mesh.label(i);
  if (label >= colormap_.getNumLabels()) {
    // Return gray to differentiate unknown labels and not having labels.
    return Color::gray(0.5);
  }
  return colormap_.getColorFromLabel(label);
}

MeshColorAdaptor::MeshColorAdaptor(const Mesh& mesh, MeshColoring::ConstPtr coloring)
    : mesh(mesh), coloring_(std::move(coloring)) {
  if (coloring_) {
    getVertexColor = [this](size_t i) {
      return coloring_->getVertexColor(this->mesh, i);
    };
  } else {
    getVertexColor = [this](size_t i) { return this->mesh.color(i); };
  }
}

Eigen::Vector3f pgmoGetVertex(const MeshColorAdaptor& mesh_adaptor,
                              size_t i,
                              kimera_pgmo::traits::VertexTraits* traits) {
  const auto c = mesh_adaptor.getVertexColor(i);
  traits->color = kimera_pgmo::traits::Color{{c.r, c.g, c.b, c.a}};
  return mesh_adaptor.mesh.pos(i);
}

size_t pgmoNumVertices(const MeshColorAdaptor& mesh_adaptor) {
  return pgmoNumVertices(mesh_adaptor.mesh);
}

size_t pgmoNumFaces(const MeshColorAdaptor& mesh_adaptor) {
  return pgmoNumFaces(mesh_adaptor.mesh);
}

kimera_pgmo::traits::Face pgmoGetFace(const MeshColorAdaptor& mesh_adaptor, size_t i) {
  return pgmoGetFace(mesh_adaptor.mesh, i);
}

Color colorFromTime(Mesh::Timestamp time, Mesh::Timestamp min, Mesh::Timestamp max) {
  if (time == 0) {
    return Color::green();
  }
  if (time <= min) {
    return Color::ironbow(0);
  } else if (time >= max) {
    return Color::ironbow(1);
  }
  const double normalized_time_stamp =
      static_cast<double>(time - min) / static_cast<double>(max - min);
  return Color::ironbow(normalized_time_stamp);
}

}  // namespace hydra
