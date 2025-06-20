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
#include <config_utilities/types/path.h>
#include <config_utilities/logging/log_to_glog.h>
#include <config_utilities/parsing/context.h>
#include <hydra/common/global_info.h>
#include <ianvs/node_handle_factory.h>
#include <ianvs/spin_functions.h>

#include "hydra_ros/hydra_ros_pipeline.h"

namespace hydra {

struct RunSettings {
  size_t robot_id = 0;
  bool exit_after_clock = false;
  bool force_shutdown = false;
  size_t print_width = 100;
  size_t print_indent = 45;
  bool print_missing = false;
  bool allow_plugins = true;
  bool verbose_plugins = false;
  bool trace_plugin_allocations = false;
  std::vector<std::string> paths;
  int config_verbosity = 1;
  bool forward_glog_to_ros = true;
  int glog_level = 0;
  int glog_verbosity = 0;
  std::filesystem::path log_path;
  hydra::DataDirectory::Config output;
};

void declare_config(RunSettings& config) {
  using namespace config;
  name("RunSettings");
  field(config.robot_id, "robot_id");
  field(config.exit_after_clock, "exit_after_clock");
  field(config.force_shutdown, "force_shutdown");
  field(config.print_width, "print_width");
  field(config.print_indent, "print_indent");
  field(config.print_missing, "print_missing");
  field(config.allow_plugins, "allow_plugins");
  field(config.verbose_plugins, "verbose_plugins");
  field(config.trace_plugin_allocations, "trace_plugin_allocations");
  field(config.paths, "paths");
  field(config.config_verbosity, "config_verbosity");
  field(config.forward_glog_to_ros, "forward_glog_to_ros");
  field(config.glog_level, "glog_level");
  field(config.glog_verbosity, "glog_verbosity");
  field<Path::Absolute>(config.log_path, "log_path");
  field(config.output, "output");
}

struct RosSink : google::LogSink {
  explicit RosSink(const rclcpp::Logger& logger) : logger_(logger) {}

  void send(google::LogSeverity severity,
            const char* /*full_filename*/,
            const char* base_filename,
            int line,
            const struct ::tm* /*time*/,
            const char* message,
            size_t message_len) override {
    std::stringstream ss;
    ss << "[" << base_filename << ":" << line << "] "
       << std::string(message, message_len);
    switch (severity) {
      case google::GLOG_WARNING:
        RCLCPP_WARN_STREAM(logger_, ss.str());
        break;
      case google::GLOG_ERROR:
        RCLCPP_ERROR_STREAM(logger_, ss.str());
        break;
      case google::GLOG_FATAL:
        RCLCPP_FATAL_STREAM(logger_, ss.str());
        break;
      case google::GLOG_INFO:
      default:
        RCLCPP_INFO_STREAM(logger_, ss.str());
        break;
    }
  }

  rclcpp::Logger logger_;
};

}  // namespace hydra

int main(int argc, char* argv[]) {
  config::initContext(argc, argv, true);
  rclcpp::init(argc, argv);

  const auto settings = config::fromContext<hydra::RunSettings>();

  FLAGS_minloglevel = settings.glog_level;
  FLAGS_v = settings.glog_verbosity;
  FLAGS_logtostderr = settings.forward_glog_to_ros ? 0 : 1;
  FLAGS_colorlogtostderr = 1;

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  auto node = std::make_shared<rclcpp::Node>("hydra_ros_node");

  std::shared_ptr<hydra::RosSink> ros_sink;
  if (settings.forward_glog_to_ros) {
    ros_sink = std::make_shared<hydra::RosSink>(node->get_logger());
    google::AddLogSink(ros_sink.get());
  }

  config::Settings().setLogger("glog");
  config::Settings().print_width = settings.print_width;
  config::Settings().print_indent = settings.print_indent;
  config::Settings().print_missing = settings.print_missing;
  config::Settings().allow_external_libraries = settings.allow_plugins;
  config::Settings().verbose_external_load = settings.verbose_plugins;
  config::Settings().print_external_allocations = settings.trace_plugin_allocations;
  [[maybe_unused]] const auto plugins = config::loadExternalFactories(settings.paths);

  ianvs::NodeHandle nh(*node);
  ianvs::NodeHandleFactory::addNode("hydra_ros_node", *node);
  hydra::GlobalInfo::instance().setForceShutdown(settings.force_shutdown);

  {  // start hydra scope
    hydra::HydraRosPipeline hydra(settings.robot_id, settings.config_verbosity);
    hydra.init();

    hydra.start();
    ianvs::spinAndWait(nh, settings.exit_after_clock);
    hydra.stop();

    const hydra::DataDirectory output(settings.log_path, settings.output);
    hydra.save(output);
  }  // end hydra scope

  if (ros_sink) {
    google::RemoveLogSink(ros_sink.get());
  }

  return 0;
}
