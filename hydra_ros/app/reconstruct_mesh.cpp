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
#include <config_utilities/config_utilities.h>
#include <config_utilities/formatting/asl.h>
#include <config_utilities/logging/log_to_glog.h>
#include <config_utilities/parsing/ros.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <hydra/reconstruction/mesh_integrator.h>
#include <hydra/reconstruction/projective_integrator.h>
#include <hydra/utils/timing_utilities.h>

#include <filesystem>

#include "hydra_ros/utils/bag_reader.h"

DEFINE_string(config, "", "config contents (in YAML)");
DEFINE_string(output_path, "", "output directory");
DEFINE_bool(show_timers, false, "print timers during reconstruction");

namespace hydra {

struct Reconstructor {
  struct Config {
    VolumetricMap::Config map;
    ProjectiveIntegratorConfig integrator;
  } const config;

  explicit Reconstructor(const Config& config)
      : config(config::checkValid(config)),
        map(config.map),
        integrator(std::make_unique<ProjectiveIntegrator>(config.integrator)) {}

  void update(const InputData& data) const {
    VLOG(5) << "processing data @ " << data.timestamp_ns;
    timing::ScopedTimer timer("update_tsdf", data.timestamp_ns, true, 1, false);
    integrator->updateMap(data, map);
  }

  void reconstruct(const std::string& output_dir) {
    if (!map.getTsdfLayer().numBlocks()) {
      LOG(ERROR) << "TSDF is empty! Not saving output";
      return;
    }

    {
      timing::ScopedTimer timer("reconstruct_mesh", 0, true, 1);
      MeshIntegratorConfig mesh_config;
      MeshIntegrator mesh_integrator(mesh_config);
      mesh_integrator.generateMesh(map, false, false);
    }

    std::filesystem::path output_path(output_dir);
    if (!std::filesystem::exists(output_path)) {
      std::filesystem::create_directories(output_path);
    }

    Mesh full_mesh;
    {
      timing::ScopedTimer timer("connect_mesh", 0, true, 1);
      // TODO(lschmid): BROKEN fix the mesh connecting. Can be implemented by adapting
      // this
      // https://github.mit.edu/SPARK/Hydra/blob/66b701702adb51a4c076ba0e26baf59462863d5a/eval/tools/compress_graph.cpp#L201-L217
      // to 'full_mesh = connectMesh(map.getMeshLayer());'
      LOG(ERROR) << "Mesh blocks  are currently not being connected!";
    }

    LOG(INFO) << "Saving mesh and tsdf to " << output_path;
    timing::ScopedTimer io_timer("save_files", 0, true, 1);
    full_mesh.save(output_path / "mesh");
    map.save(output_path / "map");
  }

  mutable VolumetricMap map;
  std::unique_ptr<ProjectiveIntegrator> integrator;
};

void declare_config(Reconstructor::Config& config) {
  using namespace config;
  name("Reconstructor::Config");
  field(config.map, "map");
  field(config.integrator, "integrator");
}

}  // namespace hydra

struct ReconstructMeshConfig {
  hydra::Reconstructor::Config reconstructor;
  hydra::BagReader::Config reader;
};

void declare_config(ReconstructMeshConfig& config) {
  using namespace config;
  name("ReconstructMeshConfig");
  field(config.reconstructor, "reconstructor");
  field(config.reader, "reader");
}

int main(int argc, char* argv[]) {
  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  config::Settings().setLogger("glog");
  config::Settings().print_width = 120;
  config::Settings().print_indent = 25;

  hydra::timing::ElapsedTimeRecorder::instance().disable_output = !FLAGS_show_timers;

  VLOG(1) << "Loading from " << FLAGS_config;
  std::filesystem::path config_path(FLAGS_config);
  CHECK(std::filesystem::exists(config_path)) << "invalid path: " << config_path;

  YAML::Node node = YAML::LoadFile(config_path);
  const auto config = config::fromYaml<ReconstructMeshConfig>(node);
  VLOG(1) << std::endl << config::toString(config);

  hydra::BagReader reader(config.reader);
  auto reconstructor = std::make_shared<hydra::Reconstructor>(config.reconstructor);
  auto sink = hydra::BagReader::Sink::fromMethod(&hydra::Reconstructor::update,
                                                 reconstructor.get());
  reader.addSink(sink);

  LOG(INFO) << "Parsing bags...";
  reader.read();
  LOG(INFO) << "Finished parsing";
  LOG(INFO) << "Reconstructing and saving mesh...";
  reconstructor->reconstruct(FLAGS_output_path);

  LOG(INFO) << "Timing: "
            << hydra::timing::ElapsedTimeRecorder::instance().getPrintableStats();
  return 0;
}
