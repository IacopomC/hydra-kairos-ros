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
#include "hydra_visualizer/scene_graph_renderer.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/ros.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>
#include <std_msgs/String.h>
#include <tf2_eigen/tf2_eigen.h>

#include "hydra_visualizer/color/colormap_utilities.h"
#include "hydra_visualizer/utils/config_manager.h"
#include "hydra_visualizer/utils/visualizer_utilities.h"

namespace hydra {

using namespace spark_dsg;
using namespace visualizer;

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

namespace {

constexpr float kFlowColorR = 1.00f;
constexpr float kFlowColorG = 0.00f;
constexpr float kFlowColorB = 0.00f;

double readDouble(const nlohmann::json& metadata,
                  const std::string& key,
                  double fallback = 0.0) {
  if (!metadata.is_object()) {
    return fallback;
  }

  const auto iter = metadata.find(key);
  if (iter == metadata.end() || !iter->is_number()) {
    return fallback;
  }

  return iter->get<double>();
}

bool readBool(const nlohmann::json& metadata,
              const std::string& key,
              bool fallback = false) {
  if (!metadata.is_object()) {
    return fallback;
  }

  const auto iter = metadata.find(key);
  if (iter == metadata.end() || !iter->is_boolean()) {
    return fallback;
  }

  return iter->get<bool>();
}

bool hasFlowDynamics(const nlohmann::json& metadata) {
  if (!metadata.is_object()) {
    return false;
  }

  const auto iter = metadata.find("flow_num_components");
  if (iter == metadata.end() || !iter->is_number()) {
    return false;
  }

  return iter->get<double>() > 0.0;
}

MarkerArray makeFlowArrowMarkers(const std_msgs::Header& header,
                                 const visualizer::StaticLayerInfo& info,
                                 const spark_dsg::SceneGraphLayer& layer,
                                 const std::string& ns) {
  MarkerArray msg;

  int marker_id = 0;
  for (const auto& [node_id, node] : layer.nodes()) {
    if (info.filter && !info.filter(*node)) {
      continue;
    }

    const auto& attrs = node->attributes();
    const auto& metadata = attrs.metadata;
    if (!metadata.is_object() || !metadata.contains("flow_components") ||
        !metadata.at("flow_components").is_array()) {
      continue;
    }

    const double confidence = std::clamp(readDouble(metadata, "flow_confidence", 1.0), 0.0, 1.0);
    const bool show_dominant_overlay = readBool(metadata, "flow_use_fixed_components", false);
    bool emitted_component_arrow = false;

    for (const auto& component : metadata.at("flow_components")) {
      if (!component.is_object()) {
        continue;
      }

      if (!component.contains("theta") || !component.contains("rho") ||
          !component.contains("weight") || !component.at("theta").is_number() ||
          !component.at("rho").is_number() || !component.at("weight").is_number()) {
        continue;
      }

      const double theta = component.at("theta").get<double>();
      const double rho = std::max(0.0, component.at("rho").get<double>());
      const double weight = std::clamp(component.at("weight").get<double>(), 0.0, 1.0);

      const double length = std::clamp(0.8 * rho * (0.25 + 0.75 * weight), 0.05, 2.0);
      if (length <= 0.0) {
        continue;
      }

      Marker marker;
      marker.header = header;
      marker.ns = ns;
      marker.id = marker_id++;
      marker.type = Marker::ARROW;
      marker.action = Marker::ADD;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.025 + 0.08 * weight;
      marker.scale.y = 0.06 + 0.18 * weight;
      marker.scale.z = 0.08 + 0.20 * weight;

      geometry_msgs::Point start;
      start.x = attrs.position.x();
      start.y = attrs.position.y();
      start.z = attrs.position.z() + info.getZOffset();
      marker.points.push_back(start);

      geometry_msgs::Point end;
      end.x = attrs.position.x() + length * std::cos(theta);
      end.y = attrs.position.y() + length * std::sin(theta);
      end.z = attrs.position.z() + info.getZOffset();
      marker.points.push_back(end);

      marker.color.r = kFlowColorR;
      marker.color.g = kFlowColorG;
      marker.color.b = kFlowColorB;
      marker.color.a = static_cast<float>(std::max(0.40, confidence * std::max(weight, 0.35)));

      msg.markers.push_back(marker);
      emitted_component_arrow = true;
    }

    if (!show_dominant_overlay) {
      continue;
    }

    const double dominant_theta = readDouble(metadata, "flow_dominant_theta", 0.0);
    const double dominant_rho = std::max(0.0, readDouble(metadata, "flow_dominant_rho", 0.0));
    const double dominant_length = std::clamp(0.8 * dominant_rho, 0.25, 2.0);
    if (dominant_length > 0.0) {
      Marker dominant_marker;
      dominant_marker.header = header;
      dominant_marker.ns = ns;
      dominant_marker.id = marker_id++;
      dominant_marker.type = Marker::ARROW;
      dominant_marker.action = Marker::ADD;
      dominant_marker.pose.orientation.w = 1.0;

      dominant_marker.scale.x = 0.055;
      dominant_marker.scale.y = 0.120;
      dominant_marker.scale.z = 0.160;

      const double z_offset = info.getZOffset() + (emitted_component_arrow ? 0.10 : 0.0);

      geometry_msgs::Point start;
      start.x = attrs.position.x();
      start.y = attrs.position.y();
      start.z = attrs.position.z() + z_offset;
      dominant_marker.points.push_back(start);

      geometry_msgs::Point end;
      end.x = attrs.position.x() + dominant_length * std::cos(dominant_theta);
      end.y = attrs.position.y() + dominant_length * std::sin(dominant_theta);
      end.z = attrs.position.z() + z_offset;
      dominant_marker.points.push_back(end);

      dominant_marker.color.r = kFlowColorR;
      dominant_marker.color.g = kFlowColorG;
      dominant_marker.color.b = kFlowColorB;
      dominant_marker.color.a = static_cast<float>(std::max(0.55, confidence));

      msg.markers.push_back(dominant_marker);
    }
  }

  return msg;
}

}  // namespace

struct MarkerNamespaces {
  static std::string layerNodeNamespace(spark_dsg::LayerId layer) {
    return std::string("layer_nodes_") + std::to_string(layer);
  }

  static std::string layerEdgeNamespace(spark_dsg::LayerId layer) {
    return std::string("layer_edges_") + std::to_string(layer);
  }

  static std::string layerLabelNamespace(spark_dsg::LayerId layer) {
    return std::string("layer_labels_") + std::to_string(layer);
  }

  static std::string layerBboxNamespace(spark_dsg::LayerId layer) {
    return std::string("layer_bounding_boxes_") + std::to_string(layer);
  }

  static std::string layerBoundaryNamespace(spark_dsg::LayerId layer) {
    return std::string("layer_polygon_boundaries_") + std::to_string(layer);
  }

  static std::string layerBoundaryEllipseNamespace(spark_dsg::LayerId layer) {
    return std::string("layer_ellipsoid_boundaries_") + std::to_string(layer);
  }

  static std::string layerBoundaryEdgeNamespace(spark_dsg::LayerId layer) {
    return std::string("layer_polygon_boundaries_edges_") + std::to_string(layer);
  }

  static std::string meshEdgeNamespace(spark_dsg::LayerId layer) {
    return std::string("mesh_edges_") + std::to_string(layer);
  }

  static std::string dynamicNodeNamespace(char layer_prefix) {
    return std::string("dynamic_nodes_") + layer_prefix;
  }

  static std::string dynamicEdgeNamespace(char layer_prefix) {
    return std::string("dynamic_edges_") + layer_prefix;
  }

  static std::string dynamicLabelNamespace(char layer_prefix) {
    return std::string("dynamic_label_") + layer_prefix;
  }
};

SceneGraphRenderer::SceneGraphRenderer(const ros::NodeHandle& nh)
    : nh_(nh), pub_(nh_.advertise<MarkerArray>("dsg_markers", 1, true)) {
  ConfigManager::init(nh_);
}

void SceneGraphRenderer::reset(const std_msgs::Header& header) {
  MarkerArray msg;
  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_.publish(msg);
  }

  // TODO(nathan) think if we actually want to do this
  // ConfigManager::reset();
}

bool SceneGraphRenderer::hasChange() const {
  return ConfigManager::instance().hasChange();
}

void SceneGraphRenderer::clearChangeFlag() {
  ConfigManager::instance().clearChangeFlags();
}

void SceneGraphRenderer::draw(const std_msgs::Header& header,
                              const DynamicSceneGraph& graph) {
  visualizer::GraphInfo info;
  const auto& manager = ConfigManager::instance();

  MarkerArray msg;
  for (auto&& [layer_id, layer] : graph.layers()) {
    const auto& conf = manager.getLayerConfig(layer_id);
    auto iter = info.layers.emplace(layer_id, conf.getInfo(graph)).first;
    drawLayer(header, iter->second, *layer, msg);
    if (iter->second.layer.visualize && iter->second.layer.draw_mesh_edges) {
      const std::string ns = MarkerNamespaces::meshEdgeNamespace(layer_id);
      tracker_.add(makeMeshEdgesMarker(header, iter->second, graph, *layer, ns), msg);
    }
  }

  for (auto&& [layer_id, sublayers] : graph.dynamicLayers()) {
    const auto& conf = manager.getDynamicLayerConfig(layer_id);
    auto iter = info.dynamic_layers.emplace(layer_id, conf.getInfo(graph)).first;
    for (auto&& [prefix, layer] : sublayers) {
      drawDynamicLayer(header, iter->second, *layer, msg);
    }
  }

  const auto static_edges = makeGraphEdgeMarkers(
      header, info, graph, graph.interlayer_edges(), "interlayer_edges_");
  tracker_.add(static_edges, msg);

  const auto dynamic_edges = makeGraphEdgeMarkers(header,
                                                  info,
                                                  graph,
                                                  graph.dynamic_interlayer_edges(),
                                                  "dynamic_interlayer_edges_");
  tracker_.add(dynamic_edges, msg);

  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_.publish(msg);
  }
}

void SceneGraphRenderer::drawLayer(const std_msgs::Header& header,
                                   const StaticLayerInfo& info,
                                   const SceneGraphLayer& layer,
                                   MarkerArray& msg) {
  if (!info.layer.visualize) {
    return;
  }

  if (info.layer.draw_frontier_ellipse) {
    info.filter = [](const SceneGraphNode& node) {
      try {
        return node.attributes<PlaceNodeAttributes>().real_place;
      } catch (const std::bad_cast&) {
        return true;
      }
    };
  }

  const auto node_ns = MarkerNamespaces::layerNodeNamespace(layer.id);
  if (layer.id != spark_dsg::DsgLayers::MESH_PLACES) {
    tracker_.add(makeLayerNodeMarkers(header, info, layer, node_ns), msg);
  } else {
    auto node_info = info;
    const auto base_filter = node_info.filter;
    node_info.filter = [base_filter](const SceneGraphNode& node) {
      if (base_filter && !base_filter(node)) {
        return false;
      }

      return !hasFlowDynamics(node.attributes().metadata);
    };

    tracker_.add(makeLayerNodeMarkers(header, node_info, layer, node_ns), msg);
  }

  if (info.layer.use_label) {
    const auto ns = MarkerNamespaces::layerLabelNamespace(layer.id);
    tracker_.add(makeLayerLabelMarkers(header, info, layer, ns), msg);
  }

  if (info.layer.use_bounding_box) {
    const auto ns = MarkerNamespaces::layerBboxNamespace(layer.id);
    try {
      tracker_.add(makeLayerBoundingBoxes(header, info, layer, ns), msg);
    } catch (const std::bad_cast& e) {
      LOG_FIRST_N(WARNING, 5) << "unable to draw bounding boxes for layer " << layer.id
                              << ": " << e.what();
    }
  }

  if (info.layer.draw_boundaries) {
    const auto ns = MarkerNamespaces::layerBoundaryNamespace(layer.id);
    const auto edge_ns = MarkerNamespaces::layerBoundaryEdgeNamespace(layer.id);
    try {
      tracker_.add(makeLayerPolygonBoundaries(header, info, layer, ns), msg);
      if (info.layer.collapse_boundary) {
        tracker_.add(makeLayerPolygonEdges(header, info, layer, edge_ns), msg);
      }
    } catch (const std::bad_cast& e) {
      LOG_FIRST_N(WARNING, 5) << "Could not draw boundaries for layer " << layer.id
                              << ": " << e.what();
    }
  }

  if (info.layer.draw_boundary_ellipse) {
    const auto ns = MarkerNamespaces::layerBoundaryEllipseNamespace(layer.id);
    try {
      tracker_.add(makeLayerEllipseBoundaries(header, info, layer, ns), msg);
    } catch (const std::bad_cast& e) {
      LOG_FIRST_N(WARNING, 5) << "Could not draw boundary ellipses for layer "
                              << layer.id << ": " << e.what();
    }
  }

  if (info.layer.draw_frontier_ellipse) {
    tracker_.add(makeEllipsoidMarkers(header, info, layer, "frontier_ns"), msg);
    info.filter = {};  // we reset the manual filter to draw edges to frontiers
  }

  const auto edge_ns = MarkerNamespaces::layerEdgeNamespace(layer.id);
  tracker_.add(makeLayerEdgeMarkers(header, info, layer, edge_ns), msg);

  if (layer.id == spark_dsg::DsgLayers::MESH_PLACES) {
    const auto ns = std::string("layer_flow_arrows_") + std::to_string(layer.id);
    tracker_.add(makeFlowArrowMarkers(header, info, layer, ns), msg);
  }
}

void SceneGraphRenderer::drawDynamicLayer(const std_msgs::Header& header,
                                          const DynamicLayerInfo& info,
                                          const DynamicSceneGraphLayer& layer,
                                          MarkerArray& msg) {
  if (!info.layer.visualize) {
    return;
  }

  const auto node_ns = MarkerNamespaces::dynamicNodeNamespace(layer.prefix);
  tracker_.add(makeDynamicNodeMarkers(header, info, layer, node_ns), msg);

  const auto edge_ns = MarkerNamespaces::dynamicEdgeNamespace(layer.prefix);
  tracker_.add(makeDynamicEdgeMarkers(header, info, layer, edge_ns), msg);

  const auto label_ns = MarkerNamespaces::dynamicLabelNamespace(layer.prefix);
  tracker_.add(makeDynamicLabelMarker(header, info, layer, label_ns), msg);
}

}  // namespace hydra
