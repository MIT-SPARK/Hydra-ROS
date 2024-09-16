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
#include "hydra_visualizer/color/colormap_utilities.h"

#include <config_utilities/config.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>
#include <config_utilities/virtual_config.h>
#include <glog/logging.h>
#include <spark_dsg/colormaps.h>

#include "hydra_visualizer/color/color_parsing.h"

namespace hydra::visualizer {

using spark_dsg::Color;

namespace {

const std::vector<Color>& lookupColormap(CategoricalPalette cmap) {
  switch (cmap) {
    case CategoricalPalette::COLORBREWER:
      return spark_dsg::colormaps::colorbrewerPalette();
    case CategoricalPalette::DISTINCT150:
    default:
      return spark_dsg::colormaps::distinct150Palette();
  }
}

std::function<Color(size_t)> lookupColormap(DiscretePalette cmap) {
  switch (cmap) {
    case DiscretePalette::COLORBREWER:
      return [](size_t id) { return spark_dsg::colormaps::colorbrewerId(id); };
    case DiscretePalette::DISTINCT150:
      return [](size_t id) { return spark_dsg::colormaps::distinct150Id(id); };
    case DiscretePalette::RAINBOW:
    default:
      return [](size_t id) { return spark_dsg::colormaps::rainbowId(id); };
  }
}

}  // namespace

std_msgs::ColorRGBA makeColorMsg(const Color& color, std::optional<double> alpha) {
  std_msgs::ColorRGBA msg;
  msg.r = static_cast<double>(color.r) / 255.0;
  msg.g = static_cast<double>(color.g) / 255.0;
  msg.b = static_cast<double>(color.b) / 255.0;
  msg.a = alpha.value_or(static_cast<double>(color.a) / 255.0);
  return msg;
}

spark_dsg::Color GrayPalette::getColor(double value) const {
  return spark_dsg::colormaps::gray(value);
}

void declare_config(GrayPalette::Config&) { config::name("GrayPalette::Config"); }

spark_dsg::Color QualityPalette::getColor(double value) const {
  return spark_dsg::colormaps::quality(value);
}

void declare_config(QualityPalette::Config&) { config::name("QualityPalette::Config"); }

spark_dsg::Color IronbowPalette::getColor(double value) const {
  return spark_dsg::colormaps::ironbow(value);
}

void declare_config(IronbowPalette::Config&) { config::name("IronbowPalette::Config"); }

spark_dsg::Color RainbowPalette::getColor(double value) const {
  return spark_dsg::colormaps::rainbow(value);
}

void declare_config(RainbowPalette::Config&) { config::name("RainbowPalette::Config"); }

spark_dsg::Color SpectrumPalette::getColor(double value) const {
  return spark_dsg::colormaps::spectrum(value, config.colors);
}

void declare_config(SpectrumPalette::Config& config) {
  using namespace config;
  name("SpectrumPalette::Config");
  field(config.colors, "colors");
}

spark_dsg::Color HLSPalette::getColor(double value) const {
  return spark_dsg::colormaps::hls(value, hls_start, hls_end);
}

void declare_config(HLSPalette::Config& config) {
  using namespace config;
  name("HLSPalette::Config");
  field(config.start, "start");
  field(config.end, "end");
}

spark_dsg::Color DivergentPalette::getColor(double value) const {
  return spark_dsg::colormaps::divergent(value,
                                         config.hue_low,
                                         config.hue_high,
                                         config.saturation,
                                         config.luminance,
                                         config.dark);
}

void declare_config(DivergentPalette::Config& config) {
  using namespace config;
  name("DivergentPalette::Config");
  field(config.hue_low, "hue_low");
  field(config.hue_high, "hue_high");
  field(config.saturation, "saturation");
  field(config.luminance, "luminance");
  field(config.dark, "dark");
}

void declare_config(RangeColormap::Config& config) {
  using namespace config;
  name("RangeColormap::Config");
  config.palette.setOptional();
  field(config.palette, "palette");
}

void declare_config(CategoricalColormap::Config& config) {
  using namespace config;
  name("CategoricalColormap::Config");
  field(config.total_classes, "total_classes");
  enum_field(config.palette,
             "palette",
             {{CategoricalPalette::COLORBREWER, "colorbrewer"},
              {CategoricalPalette::DISTINCT150, "distinct150"}});
  field(config.default_color, "default_color");
}

void declare_config(DiscreteColormap::Config& config) {
  using namespace config;
  name("DiscreteColormap::Config");
  enum_field(config.palette,
             "palette",
             {{DiscretePalette::COLORBREWER, "colorbrewer"},
              {DiscretePalette::DISTINCT150, "distinct150"},
              {DiscretePalette::RAINBOW, "rainbow"}});
}

RangeColormap::RangeColormap(const Config& config)
    : config(config::checkValid(config)), palette_(config.palette.create()) {}

Color RangeColormap::getColor(double value, double min, double max) const {
  const auto ratio = std::clamp((value - min) / (max - min), 0.0, 1.0);
  return palette_ ? palette_->getColor(ratio) : spark_dsg::colormaps::ironbow(ratio);
}

DiscreteColormap::DiscreteColormap() : DiscreteColormap(Config()) {}

DiscreteColormap::DiscreteColormap(const Config& config)
    : config(config::checkValid(config)), colormap(lookupColormap(config.palette)) {}

Color DiscreteColormap::getColor(size_t id) const { return colormap(id); }

CategoricalColormap::CategoricalColormap() : CategoricalColormap(Config()) {}

CategoricalColormap::CategoricalColormap(const Config& config)
    : config(config::checkValid(config)),
      colors(lookupColormap(config.palette)),
      total_classes(config.total_classes ? config.total_classes : colors.size()) {
  LOG_IF(WARNING, total_classes > colors.size())
      << "Colormap too small for number of labels: " << total_classes
      << " (colors: " << colors.size() << ")";
}

Color CategoricalColormap::getColor(size_t category) const {
  const bool color_out_of_bounds = total_classes > 0 && category >= total_classes;
  if (color_out_of_bounds || category >= colors.size()) {
    return config.default_color;
  }

  return colors.at(category);
}

}  // namespace hydra::visualizer
