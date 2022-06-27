#include <hdl_graph_slam/graph_edit_window.hpp>

#include <sstream>
#include <g2o/core/optimizable_graph.h>
#include <g2o/edge_se3_priorxyz.hpp>

namespace hdl_graph_slam {

GraphEditWindow::GraphEditWindow(std::shared_ptr<InteractiveGraphView>& graph) : show_window(false), selected_vertex(0), graph(graph) {}
GraphEditWindow::~GraphEditWindow() {}

void GraphEditWindow::draw_ui() {
  if(!show_window) {
    return;
  }

  ImGui::Begin("graph edit", &show_window, ImGuiWindowFlags_AlwaysAutoResize);

  ImGuiTabBarFlags flags = ImGuiTabBarFlags_None;
  if(ImGui::BeginTabBar("tabbar", flags)) {
    if(ImGui::BeginTabItem("Fixed vertices")) {
      g2o::VertexSE3* anchor_node = dynamic_cast<g2o::VertexSE3*>(graph->graph->vertex(graph->anchor_node_id()));
      if(anchor_node) {
        bool fixed = anchor_node->fixed();
        if(ImGui::Checkbox("Anchor Fixed", &fixed)) {
          anchor_node->setFixed(fixed);
        }
      } else {
        ImGui::Text("warning: No anchor node created!!");
      }

      bool fixed_vertex_exists = false;
      for(const auto& vertex : graph->graph->vertices()) {
        auto v = dynamic_cast<g2o::OptimizableGraph::Vertex*>(vertex.second);
        assert(v != nullptr);

        if(!v->fixed() || v->id() == graph->anchor_node_id()) {
          continue;
        }

        std::stringstream sst;
        sst << "Vertex " << v->id();

        if(ImGui::Button(("Unfix##" + std::to_string(v->id())).c_str())) {
          v->setFixed(false);
        }
        ImGui::SameLine();
        ImGui::Text("Vertex %d", v->id());
        fixed_vertex_exists = true;
      }

      if(!fixed_vertex_exists) {
        ImGui::Text("No fixed vertices!!");
      }

      ImGui::EndTabItem();
    }

    if(ImGui::BeginTabItem("Some fancy function")) {
      for(const auto& edge : graph->graph->edges()) {
        auto e = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);
        assert(e != nullptr);
      }

      ImGui::Text("hello");
      ImGui::EndTabItem();
    }

    if(ImGui::BeginTabItem("Gps vertices")) {
      std::map<long, g2o::EdgeSE3PriorXYZ*> gps_edges;
      for(const auto& edge : graph->graph->edges()) {
        auto e = dynamic_cast<g2o::EdgeSE3PriorXYZ*>(edge);
        if(e)
          gps_edges[edge->vertices()[0]->id()] = e;
      }

      ImGui::Text("gps edges %lu", gps_edges.size());

      ImGui::Separator();
      ImGui::InputDouble("gps_edge_stddev_xy", &gps_edge_stddev_xy);
      ImGui::InputDouble("gps_edge_stddev_z", &gps_edge_stddev_z);
      ImGui::InputDouble("gps_edge_multiplier", &gps_edge_multiplier);
      ImGui::Checkbox("only2d", &only2d);
      
      char buff[16];
      strcpy(buff, kernel_type.data());
      ImGui::InputText("robust kernel", buff, 16);
      ImGui::InputDouble("robust kernel delta", &kernel_delta);
      kernel_type = std::string(buff);
      
      if(ImGui::Button("add gps edges")) {
        for(const auto& key : graph->keyframes) {
          if(key.second->utm_coord && gps_edges.count(key.second->node->id()) == 0) {
            auto p = key.second->utm_coord.value();
            if(only2d)
              p.z() = 0;
            graph->add_edge_prior_xyz(key.second, p, gps_edge_stddev_xy*gps_edge_multiplier, gps_edge_stddev_z*gps_edge_multiplier);
          }
        }
      }
      if(ImGui::Button("update edges stddev")) {
        for(auto& e : gps_edges) {
          Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
            information_matrix.block<2, 2>(0, 0) /= gps_edge_stddev_xy*gps_edge_multiplier;
            information_matrix(2, 2) /= gps_edge_stddev_z*gps_edge_multiplier;
          e.second->information() = information_matrix;

          graph->apply_robust_kernel(e.second, kernel_type, kernel_delta);
        }

      }
    
      ImGui::EndTabItem();
    }


    ImGui::EndTabBar();
  }

  ImGui::End();
}

}  // namespace hdl_graph_slam