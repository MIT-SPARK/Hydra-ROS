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
#include "hydra_ros/utils/node_utilities.h"

#include <glog/logging.h>
#include <hydra/utils/timing_utilities.h>
#include <hydra/common/hydra_config.h>

namespace hydra {

using timing::ElapsedTimeRecorder;

ExitMode getExitMode(const ros::NodeHandle& nh) {
  std::string exit_mode_str = "NORMAL";
  nh.getParam("exit_mode", exit_mode_str);

  if (exit_mode_str == "CLOCK") {
    return ExitMode::CLOCK;
  } else if (exit_mode_str == "SERVICE") {
    return ExitMode::SERVICE;
  } else if (exit_mode_str == "NORMAL") {
    return ExitMode::NORMAL;
  } else {
    ROS_WARN_STREAM("Unrecognized option: " << exit_mode_str
                                            << ". Defaulting to NORMAL");
    return ExitMode::NORMAL;
  }
}

void spinWhileClockPresent() {
  ros::WallRate r(50);
  ROS_INFO("Waiting for bag to start");
  while (ros::ok() && !haveClock()) {
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Running...");
  while (ros::ok() && haveClock()) {
    ros::spinOnce();
    r.sleep();
  }

  ros::spinOnce();  // make sure all the callbacks are processed
  ROS_WARN("Exiting!");
}

void spinUntilExitRequested() {
  ServiceFunctor functor;

  ros::NodeHandle nh("~");
  ros::ServiceServer service =
      nh.advertiseService("shutdown", &ServiceFunctor::callback, &functor);

  ros::WallRate r(50);
  ROS_INFO("Running...");
  while (ros::ok() && !functor.should_exit) {
    ros::spinOnce();
    r.sleep();
  }

  ros::spinOnce();  // make sure all the callbacks are processed
  ROS_WARN("Exiting!");
}

std::string configureTimers(const ros::NodeHandle& nh) {
  std::string dsg_output_path = "";
  nh.getParam("log_path", dsg_output_path);

  nh.getParam("timing_disabled", ElapsedTimeRecorder::instance().timing_disabled);
  nh.getParam("disable_timer_output", ElapsedTimeRecorder::instance().disable_output);

  bool log_timing_incrementally = false;
  nh.getParam("log_timing_incrementally", log_timing_incrementally);
  if (log_timing_incrementally && dsg_output_path != "") {
    ElapsedTimeRecorder::instance().setupIncrementalLogging(dsg_output_path);
  }

  return dsg_output_path;
}

void spinAndWait(const ros::NodeHandle& nh) {
  const auto exit_mode = getExitMode(nh);
  switch (exit_mode) {
    case ExitMode::CLOCK:
      spinWhileClockPresent();
      break;
    case ExitMode::SERVICE:
      spinUntilExitRequested();
      break;
    case ExitMode::NORMAL:
    default:
      ros::spin();
      break;
  }
}

void saveTimingInformation(const std::string& dsg_output_path) {
  if (dsg_output_path.empty()) {
    return;
  }

  LOG(INFO) << "[DSG Node] saving timing information to " << dsg_output_path;
  const ElapsedTimeRecorder& timer = ElapsedTimeRecorder::instance();
  timer.logAllElapsed(dsg_output_path);
  timer.logStats(dsg_output_path);
  LOG(INFO) << "[DSG Node] Saved timing information to " << dsg_output_path;
}

void parseObjectNamesFromRos(const ros::NodeHandle& node_handle) {
  XmlRpc::XmlRpcValue label_names;
  node_handle.getParam("label_names", label_names);
  if (label_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    LOG(WARNING) << "Failed to parse object label names";
  }

  std::map<uint8_t, std::string> name_map;
  for (int i = 0; i < label_names.size(); ++i) {
    const auto& label_name_pair = label_names[i];
    if (label_name_pair.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      LOG(WARNING) << "Label names not formatted correctly";
      continue;
    }

    const auto label = static_cast<int>(label_name_pair["label"]);
    const auto name = static_cast<std::string>(label_name_pair["name"]);
    name_map.emplace(label, name);
  }

  HydraConfig::instance().setLabelToNameMap(name_map);
}

}  // namespace hydra
