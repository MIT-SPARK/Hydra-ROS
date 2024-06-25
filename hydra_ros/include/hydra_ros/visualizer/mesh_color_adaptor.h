/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#pragma once

#include <kimera_pgmo/mesh_traits.h>
#include <spark_dsg/mesh.h>

namespace hydra {

class SemanticColorMap;

/**
 * @brief Functor to color a mesh.
 */
struct MeshColoring {
  using Ptr = std::shared_ptr<MeshColoring>;
  using ConstPtr = std::shared_ptr<const MeshColoring>;

  virtual ~MeshColoring() = default;

  /**
   * @brief Function to color a mesh. Return the color of the i-th vertex. the base mesh
   * colorig will return the stored color of the mesh if it exists.
   */
  virtual spark_dsg::Color getVertexColor(const spark_dsg::Mesh& mesh,
                                          size_t i) const = 0;
};

/**
 * @brief Functor to color a mesh with a single color.
 */
struct UniformMeshColoring : public MeshColoring {
  explicit UniformMeshColoring(spark_dsg::Color color) : color_(std::move(color)) {}
  virtual ~UniformMeshColoring() = default;

  spark_dsg::Color getVertexColor(const spark_dsg::Mesh&, size_t) const override {
    return color_;
  }

 private:
  const spark_dsg::Color color_;
};

/**
 * @brief Functor to color a mesh based on the semantic label of the vertex.
 */
struct SemanticMeshColoring : public MeshColoring {
  explicit SemanticMeshColoring(const SemanticColorMap& colormap);
  virtual ~SemanticMeshColoring() = default;

  spark_dsg::Color getVertexColor(const spark_dsg::Mesh& mesh, size_t i) const override;

 private:
  const SemanticColorMap& colormap_;
};

/**
 * @brief Functor to color a mesh based on the first time a vertex was seen. The color
 * ranges from dark (early first seen) to light (late first seen), with unitialized
 * vertices being shown in green.
 *
 */
struct FirstSeenMeshColoring : public MeshColoring {
  explicit FirstSeenMeshColoring(const spark_dsg::Mesh& mesh);
  FirstSeenMeshColoring(spark_dsg::Mesh::Timestamp min, spark_dsg::Mesh::Timestamp max);
  virtual ~FirstSeenMeshColoring() = default;

  spark_dsg::Color getVertexColor(const spark_dsg::Mesh& mesh, size_t i) const override;

 private:
  spark_dsg::Mesh::Timestamp min_;
  spark_dsg::Mesh::Timestamp max_;
};

/**
 * @brief Functor to color a mesh based on the last time a vertex was seen. The color
 * ranges from dark (early last seen) to light (late last seen), with unitialized
 * vertices being shown in green.
 */
struct LastSeenMeshColoring : public MeshColoring {
  explicit LastSeenMeshColoring(const spark_dsg::Mesh& mesh);
  LastSeenMeshColoring(spark_dsg::Mesh::Timestamp min, spark_dsg::Mesh::Timestamp max);
  virtual ~LastSeenMeshColoring() = default;

  spark_dsg::Color getVertexColor(const spark_dsg::Mesh& mesh, size_t i) const override;

 private:
  spark_dsg::Mesh::Timestamp min_;
  spark_dsg::Mesh::Timestamp max_;
};

/**
 * @brief Functor to color a mesh based on the duration a vertex was seen. The color
 * ranges from dark (short duration) to light (long duration), with invalid durations
 * being shown in green.
 */
struct SeenDurationMeshColoring : public MeshColoring {
  explicit SeenDurationMeshColoring(const spark_dsg::Mesh& mesh);
  explicit SeenDurationMeshColoring(spark_dsg::Mesh::Timestamp max);
  virtual ~SeenDurationMeshColoring() = default;

  spark_dsg::Color getVertexColor(const spark_dsg::Mesh& mesh, size_t i) const override;

 private:
  spark_dsg::Mesh::Timestamp max_;
};

/**
 * @brief Utility class to color a mesh based on a mesh coloring for visualization.
 */
struct MeshColorAdaptor {
  /**
   * @brief Function to color a mesh. Return the color of the i-th vertex. If no
   * coloring is set, the color stored in the mesh is used.
   */
  explicit MeshColorAdaptor(const spark_dsg::Mesh& mesh,
                            MeshColoring::ConstPtr coloring = nullptr);
  virtual ~MeshColorAdaptor() = default;

  std::function<spark_dsg::Color(size_t)> getVertexColor;

  const spark_dsg::Mesh& mesh;

 private:
  const MeshColoring::ConstPtr coloring_;
};

Eigen::Vector3f pgmoGetVertex(const MeshColorAdaptor& mesh_adaptor,
                              size_t i,
                              kimera_pgmo::traits::VertexTraits* traits);

size_t pgmoNumVertices(const MeshColorAdaptor& mesh_adaptor);

size_t pgmoNumFaces(const MeshColorAdaptor& mesh_adaptor);

kimera_pgmo::traits::Face pgmoGetFace(const MeshColorAdaptor& mesh_adaptor, size_t i);

spark_dsg::Color colorFromTime(spark_dsg::Mesh::Timestamp time,
                               spark_dsg::Mesh::Timestamp min,
                               spark_dsg::Mesh::Timestamp max);

}  // namespace hydra
