#include "hydra_ros/visualizer/mesh_color_adaptor.h"

#include <spark_dsg/pgmo_mesh_traits.h>

namespace hydra {

MeshColorAdaptor::MeshColorAdaptor(const spark_dsg::Mesh& mesh,
                                   ColoringFunction coloring)
    : mesh(mesh), coloring(std::move(coloring)) {}

Eigen::Vector3f pgmoGetVertex(const MeshColorAdaptor& mesh_adaptor,
                              size_t i,
                              std::optional<kimera_pgmo::traits::Color>* color,
                              std::optional<uint8_t>* alpha,
                              std::optional<uint64_t>*,
                              std::optional<uint32_t>*) {
  const auto c = mesh_adaptor.coloring(mesh_adaptor.mesh, i);
  *color = kimera_pgmo::traits::Color{{c.r, c.g, c.b}};
  *alpha = c.a;
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

}  // namespace hydra