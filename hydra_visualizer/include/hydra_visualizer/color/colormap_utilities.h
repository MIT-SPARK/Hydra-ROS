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
#include <config_utilities/virtual_config.h>
#include <spark_dsg/color.h>
#include <std_msgs/ColorRGBA.h>

namespace hydra::visualizer {

std_msgs::ColorRGBA makeColorMsg(const spark_dsg::Color& color,
                                 std::optional<double> alpha = std::nullopt);

struct ContinuousPalette {
  virtual ~ContinuousPalette() = default;
  virtual spark_dsg::Color getColor(double value) const = 0;
  spark_dsg::Color operator()(double value) const { return getColor(value); }
};

#define REGISTER_CONTINUOUS(map_type, name) \
  inline static const auto reg =            \
      config::RegistrationWithConfig<ContinuousPalette, map_type, Config>(name)

struct GrayPalette : ContinuousPalette {
  struct Config {};
  explicit GrayPalette(const Config& = {}) {}
  spark_dsg::Color getColor(double value) const override;
  REGISTER_CONTINUOUS(GrayPalette, "gray");
};

void declare_config(GrayPalette::Config& config);

struct QualityPalette : ContinuousPalette {
  struct Config {};
  explicit QualityPalette(const Config& = {}) {}
  spark_dsg::Color getColor(double value) const override;
  REGISTER_CONTINUOUS(QualityPalette, "quality");
};

void declare_config(QualityPalette::Config& config);

struct IronbowPalette : ContinuousPalette {
  struct Config {};
  explicit IronbowPalette(const Config& = {}) {}
  spark_dsg::Color getColor(double value) const override;
  REGISTER_CONTINUOUS(IronbowPalette, "ironbow");
};

void declare_config(IronbowPalette::Config& config);

struct RainbowPalette : ContinuousPalette {
  struct Config {};
  explicit RainbowPalette(const Config& = {}) {}
  spark_dsg::Color getColor(double value) const override;
  REGISTER_CONTINUOUS(RainbowPalette, "rainbow");
};

void declare_config(RainbowPalette::Config& config);

struct SpectrumPalette : ContinuousPalette {
  struct Config {
    std::vector<spark_dsg::Color> colors{spark_dsg::Color(), spark_dsg::Color::red()};
  } const config;

  SpectrumPalette() : config(Config()) {}
  explicit SpectrumPalette(const Config& config) : config(config) {}
  spark_dsg::Color getColor(double value) const override;
  REGISTER_CONTINUOUS(SpectrumPalette, "spectrum");
};

void declare_config(SpectrumPalette::Config& config);

struct HLSPalette : ContinuousPalette {
  struct Config {
    spark_dsg::Color start;
    spark_dsg::Color end = spark_dsg::Color::red();
  } const config;

  explicit HLSPalette(const Config& config)
      : config(config), hls_start(config.start.toHLS()), hls_end(config.end.toHLS()) {}
  HLSPalette() : HLSPalette(Config()) {}
  spark_dsg::Color getColor(double value) const override;

  const std::array<float, 3> hls_start;
  const std::array<float, 3> hls_end;
  REGISTER_CONTINUOUS(HLSPalette, "hls");
};

void declare_config(HLSPalette::Config& config);

struct DivergentPalette : ContinuousPalette {
  struct Config {
    float hue_low = 0.6;
    float hue_high = 0.0;
    float saturation = 0.65;
    float luminance = 0.5;
    bool dark = true;
  } const config;

  explicit DivergentPalette(const Config& config) : config(config) {}
  DivergentPalette() : DivergentPalette(Config()) {}
  spark_dsg::Color getColor(double value) const override;

  REGISTER_CONTINUOUS(DivergentPalette, "divergent");
};

void declare_config(DivergentPalette::Config& config);

#undef REGISTER_CONTINUOUS

/**
 * @brief Possible color palettes for unbounded discrete sets
 */
enum class DiscretePalette {
  COLORBREWER,
  DISTINCT150,
  RAINBOW,
};

/**
 * @brief Possible color palettes for categorical colormaps
 */
enum class CategoricalPalette {
  COLORBREWER,
  DISTINCT150,
};

/**
 * @brief Colormap for a continuous range of values
 */
struct RangeColormap {
  struct Config {
    config::VirtualConfig<ContinuousPalette> palette;
  } const config;

  explicit RangeColormap(const Config& config);
  spark_dsg::Color getColor(double value, double min, double max) const;
  spark_dsg::Color operator()(double value, double min, double max) const {
    return getColor(value, min, max);
  }

 private:
  std::unique_ptr<ContinuousPalette> palette_;
};

void declare_config(RangeColormap::Config& config);

/**
 * @brief Colormap for a unbounded discrete set of values
 */
struct DiscreteColormap {
  struct Config {
    DiscretePalette palette = DiscretePalette::RAINBOW;
  } const config;

  DiscreteColormap();
  explicit DiscreteColormap(const Config& config);
  spark_dsg::Color getColor(size_t value) const;
  spark_dsg::Color operator()(size_t value) const { return getColor(value); }

  const std::function<spark_dsg::Color(size_t)> colormap;
};

void declare_config(DiscreteColormap::Config& config);

/**
 * @brief Colormap for a fixed number of categories
 */
struct CategoricalColormap {
  struct Config {
    size_t total_classes = 0;
    CategoricalPalette palette = CategoricalPalette::DISTINCT150;
    spark_dsg::Color default_color = spark_dsg::Color::gray();
  } const config;

  CategoricalColormap();
  explicit CategoricalColormap(const Config& config);
  spark_dsg::Color getColor(size_t category) const;
  spark_dsg::Color operator()(size_t category) const { return getColor(category); }

  const std::vector<spark_dsg::Color>& colors;
  const size_t total_classes;
};

void declare_config(CategoricalColormap::Config& config);

}  // namespace hydra::visualizer
