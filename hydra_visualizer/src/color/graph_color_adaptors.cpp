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
#include "hydra_visualizer/color/graph_color_adaptors.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include <cmath>

#include "hydra_visualizer/color/color_parsing.h"

namespace hydra {

using namespace spark_dsg;

void declare_config(NodeColorAdaptor::Config& config) {
  using namespace config;
  name("NodeColorAdaptor::Config");
  field(config.default_color, "default_color");
}

NodeColorAdaptor::NodeColorAdaptor(const Config& config) : config(config) {}

Color NodeColorAdaptor::getColor(const DynamicSceneGraph&,
                                 const SceneGraphNode& node) const {
  try {
    return node.attributes<SemanticNodeAttributes>().color;
  } catch (const std::bad_cast&) {
    return config.default_color;
  }
}

void declare_config(UniformColorAdaptor::Config& config) {
  using namespace config;
  name("UniformColorAdaptor::Config");
  field(config.color, "color");
}

UniformColorAdaptor::UniformColorAdaptor(const Config& config) : config(config) {}

Color UniformColorAdaptor::getColor(const DynamicSceneGraph&,
                                    const SceneGraphNode& /* node */) const {
  return config.color;
}

LabelColorAdaptor::LabelColorAdaptor(const Config& config)
    : config(config::checkValid(config)), colormap_(config.colormap) {}

Color LabelColorAdaptor::getColor(const DynamicSceneGraph&,
                                  const SceneGraphNode& node) const {
  SemanticLabel label = SemanticNodeAttributes::NO_SEMANTIC_LABEL;
  try {
    label = node.attributes<SemanticNodeAttributes>().semantic_label;
  } catch (const std::bad_cast&) {
    VLOG(5) << "Node " << NodeSymbol(node.id).getLabel() << " has no label!";
  }

  return colormap_.getColor(label);
}

void declare_config(LabelColorAdaptor::Config& config) {
  using namespace config;
  name("LabelColorAdaptor::Config");
  field(config.colormap, "colormap");
}

IdColorAdaptor::IdColorAdaptor(const Config& config)
    : config(config::checkValid(config)), colormap_(config.colormap) {}

Color IdColorAdaptor::getColor(const DynamicSceneGraph&,
                               const SceneGraphNode& node) const {
  return colormap_.getColor(NodeSymbol(node.id).categoryId());
}

void declare_config(IdColorAdaptor::Config& config) {
  using namespace config;
  name("IdColorAdaptor::Config");
  field(config.colormap, "colormap");
}

TemporalPlaceColorAdaptor::TemporalPlaceColorAdaptor(const Config& config)
    : config(config::checkValid(config)),
  fallback_(config.fallback_adaptor ? config.fallback_adaptor.create() : nullptr) {}

Color TemporalPlaceColorAdaptor::getColor(const DynamicSceneGraph& graph,
                                          const SceneGraphNode& node) const {
  const auto& metadata = node.attributes().metadata;
  if (!metadata.contains(config.occupancy_key)) {
    return fallback_ ? fallback_->getColor(graph, node) : Color();
  }

  const auto value = metadata.at(config.occupancy_key);
  if (!value.is_number()) {
    return fallback_ ? fallback_->getColor(graph, node) : Color();
  }

  // Colour by the presence the occupancy induces right now, which lands in
  // [0, 1] by construction and keeps busier places redder.
  const double occupancy = std::max(0.0, value.get<double>());
  const float p =
      std::clamp(static_cast<float>(1.0 - std::exp(-occupancy)), 0.0f, 1.0f);
  Color color;
  color.r = static_cast<uint8_t>((1.0f - p) * config.low_color.r + p * config.high_color.r);
  color.g = static_cast<uint8_t>((1.0f - p) * config.low_color.g + p * config.high_color.g);
  color.b = static_cast<uint8_t>((1.0f - p) * config.low_color.b + p * config.high_color.b);

  float alpha = config.max_alpha;
  if (config.use_confidence_alpha && metadata.contains(config.confidence_key)) {
    const auto confidence = metadata.at(config.confidence_key);
    if (confidence.is_number()) {
      const float c = std::clamp(static_cast<float>(confidence.get<double>()), 0.0f, 1.0f);
      alpha = config.min_alpha + (config.max_alpha - config.min_alpha) * c;
    }
  }

  alpha = std::clamp(alpha, 0.0f, 1.0f);
  color.a = static_cast<uint8_t>(alpha * 255.0f);
  return color;
}

void TemporalPlaceColorAdaptor::setGraph(const DynamicSceneGraph& graph, LayerId layer) {
  if (fallback_) {
    fallback_->setGraph(graph, layer);
  }
}

void declare_config(TemporalPlaceColorAdaptor::Config& config) {
  using namespace config;
  name("TemporalPlaceColorAdaptor::Config");
  config.fallback_adaptor.setOptional();
  field(config.fallback_adaptor, "fallback_adaptor");
  field(config.low_color, "low_color");
  field(config.high_color, "high_color");
  field(config.occupancy_key, "occupancy_key");
  field(config.use_confidence_alpha, "use_confidence_alpha");
  field(config.confidence_key, "confidence_key");
  field(config.min_alpha, "min_alpha");
  check(config.min_alpha, GE, 0.0f, "min_alpha");
  check(config.min_alpha, LE, 1.0f, "min_alpha");
  field(config.max_alpha, "max_alpha");
  check(config.max_alpha, GE, 0.0f, "max_alpha");
  check(config.max_alpha, LE, 1.0f, "max_alpha");
  check(config.max_alpha, GE, config.min_alpha, "max_alpha_ge_min_alpha");
}

ParentColorAdaptor::ParentColorAdaptor(const Config& config)
    : config(config::checkValid(config)),
      parent_adaptor_(config.parent_adaptor.create()) {}

Color ParentColorAdaptor::getColor(const DynamicSceneGraph& graph,
                                   const SceneGraphNode& node) const {
  auto parent = node.getParent();
  if (!parent || !parent_adaptor_) {
    return config.default_color;
  }

  return parent_adaptor_->getColor(graph, graph.getNode(*parent));
}

void declare_config(ParentColorAdaptor::Config& config) {
  using namespace config;
  name("ParentColorAdaptor::Config");
  config.parent_adaptor.setOptional();
  field(config.parent_adaptor, "parent_adaptor");
  field(config.default_color, "default_color");
}

bool IsActiveFunctor::eval(const DynamicSceneGraph&, const SceneGraphNode& node) const {
  return node.attributes().is_active;
}

bool NeedsCleanupFunctor::eval(const DynamicSceneGraph&,
                               const SceneGraphNode& node) const {
  return node.attributes<Place2dNodeAttributes>().need_cleanup_splitting;
}

bool HasActiveMeshFunctor::eval(const DynamicSceneGraph&,
                                const SceneGraphNode& node) const {
  return node.attributes<Place2dNodeAttributes>().has_active_mesh_indices;
}

StatusColorAdaptor::StatusColorAdaptor(const Config& config)
    : config(config), functor_(config::create<StatusFunctor>(config.status_functor)) {
  CHECK(functor_) << "invalid functor type: " << config.status_functor;
}

Color StatusColorAdaptor::getColor(const DynamicSceneGraph& graph,
                                   const SceneGraphNode& node) const {
  bool status = false;
  try {
    status = functor_->eval(graph, node);
  } catch (const std::bad_cast& e) {
    LOG_FIRST_N(ERROR, 1) << "Status functor unable to cast correctly: " << e.what();
  }

  return status ? config.true_color : config.false_color;
}

void declare_config(StatusColorAdaptor::Config& config) {
  using namespace config;
  name("StatusColorAdaptor::Config");
  field(config.true_color, "true_color");
  field(config.false_color, "false_color");
  field(config.status_functor, "status_functor");
}

PrefixColorAdaptor::PrefixColorAdaptor(const Config& config)
    : config(config), colormap_(config.colormap) {}

Color PrefixColorAdaptor::getColor(const DynamicSceneGraph&,
                                   const SceneGraphNode& node) const {
  // distance is measured from first relatively readable character prefix
  const auto prefix = LayerPrefix::fromId(node.id).key();
  return colormap_.getColor(static_cast<int>(prefix - '0'));
}

void declare_config(PrefixColorAdaptor::Config& config) {
  using namespace config;
  name("PrefixColorAdaptor::Config");
  field(config.colormap, "colormap");
}

FrontierColorAdaptor::FrontierColorAdaptor(const Config& config) : config(config) {}

Color FrontierColorAdaptor::getColor(const DynamicSceneGraph&,
                                     const SceneGraphNode& node) const {
  try {
    const auto& attrs = node.attributes<FrontierNodeAttributes>();
    if (attrs.real_place) {
      return Color();
    }

    if (attrs.is_predicted) {
      return config.predicted;
    }

    return attrs.active_frontier ? config.active : config.archived;
  } catch (const std::bad_cast&) {
    return Color();
  }
}

void declare_config(FrontierColorAdaptor::Config& config) {
  using namespace config;
  name("FrontierColorAdaptor::Config");
  field(config.real, "real");
  field(config.predicted, "predicted");
  field(config.active, "active");
  field(config.archived, "archived");
}

double DistanceFunctor::eval(const DynamicSceneGraph&,
                             const SceneGraphNode& node) const {
  return node.attributes<PlaceNodeAttributes>().distance;
}

double LastUpdatedFunctor::eval(const DynamicSceneGraph&,
                                const SceneGraphNode& node) const {
  return node.attributes().last_update_time_ns;
}

ValueColorAdaptor::ValueColorAdaptor(const Config& config)
    : config(config),
      min_value_(0.0),
      max_value_(1.0),
      functor_(config::create<ValueFunctor>(config.value_functor)),
      colormap_(config.colormap) {
  CHECK(functor_) << "invalid functor type: " << config.value_functor;
}

void ValueColorAdaptor::setGraph(const DynamicSceneGraph& graph, LayerId layer) {
  if (!functor_) {
    return;
  }

  bool is_first = true;
  try {
    for (const auto& [node_id, node] : graph.getLayer(layer).nodes()) {
      const auto value = functor_->eval(graph, *node);
      if (is_first) {
        min_value_ = value;
        max_value_ = value;
        is_first = false;
      } else {
        min_value_ = std::min(value, min_value_);
        max_value_ = std::max(value, max_value_);
      }
    }
  } catch (const std::exception& e) {
    LOG_FIRST_N(ERROR, 1) << "Value functor unable to evaluate: " << e.what();
  }
}

Color ValueColorAdaptor::getColor(const DynamicSceneGraph& graph,
                                  const SceneGraphNode& node) const {
  try {
    return colormap_.getColor(functor_->eval(graph, node), min_value_, max_value_);
  } catch (const std::exception& e) {
    LOG_FIRST_N(ERROR, 1) << "Value functor unable to evaluate: " << e.what();
    return Color();
  }
}

void declare_config(ValueColorAdaptor::Config& config) {
  using namespace config;
  name("ValueColorAdaptor::Config");
  field(config.colormap, "colormap");
  field(config.value_functor, "value_functor");
}

}  // namespace hydra
