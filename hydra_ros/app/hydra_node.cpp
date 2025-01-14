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
#include <config_utilities/external_registry.h>
#include <config_utilities/formatting/asl.h>
#include <config_utilities/logging/log_to_glog.h>
#include <config_utilities/parsing/ros.h>
#include <hydra/common/global_info.h>

#include "hydra_ros/hydra_ros_pipeline.h"
#include "hydra_ros/utils/node_utilities.h"
#include "hydra_ros/utils/node_handle_factory.h"


namespace hydra {

struct RunSettings {
  size_t robot_id = 0;
  bool force_shutdown = false;
  size_t print_width = 100;
  size_t print_indent = 45;
  bool print_missing = false;
  bool allow_plugins = true;
  bool verbose_plugins = false;
  bool trace_plugin_allocations = false;
  std::vector<std::string> paths;
};

void declare_config(RunSettings& config) {
  using namespace config;
  name("RunSettings");
  field(config.robot_id, "robot_id");
  field(config.force_shutdown, "force_shutdown");
  field(config.print_width, "print_width");
  field(config.print_indent, "print_indent");
  field(config.print_missing, "print_missing");
  field(config.allow_plugins, "allow_plugins");
  field(config.verbose_plugins, "verbose_plugins");
  field(config.trace_plugin_allocations, "trace_plugin_allocations");
  field(config.paths, "paths");
}

}  // namespace hydra

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "hydra_node");
  ros::NodeHandle nh("~");

  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  const auto settings = config::fromRos<hydra::RunSettings>(nh);

  config::Settings().setLogger("glog");
  config::Settings().print_width = settings.print_width;
  config::Settings().print_indent = settings.print_indent;
  config::Settings().print_missing = settings.print_missing;
  config::Settings().allow_external_libraries = settings.allow_plugins;
  config::Settings().verbose_external_load = settings.verbose_plugins;
  config::Settings().print_external_allocations = settings.trace_plugin_allocations;
  const auto plugins = config::loadExternalFactories(settings.paths);

  hydra::GlobalInfo::instance().setForceShutdown(settings.force_shutdown);

  {  // start hydra scope
    hydra::HydraRosPipeline hydra(nh, settings.robot_id);
    hydra.init();

    hydra.start();
    hydra::spinAndWait(nh);
    hydra.stop();
    hydra.save();
    // TODO(nathan) save full config
    hydra::GlobalInfo::exit();
  }  // end hydra scope

  return 0;
}
