#ifndef HDL_GRAPH_SLAM_GRAPH_EDIT_WINDOW_HPP
#define HDL_GRAPH_SLAM_GRAPH_EDIT_WINDOW_HPP

#include <memory>

#include <imgui.h>
#include <guik/gl_canvas.hpp>
#include <hdl_graph_slam/view/interactive_graph_view.hpp>


namespace hdl_graph_slam {

class GraphEditWindow {
public:
  GraphEditWindow(std::shared_ptr<InteractiveGraphView>& graph);
  ~GraphEditWindow();

  void show() { show_window = true; }
  void draw_ui();

private:
  bool show_window;
  int selected_vertex;
  std::shared_ptr<InteractiveGraphView>& graph;

  bool only2d = true;
  double gps_edge_stddev_xy = 10.0;
  double gps_edge_stddev_z = 100.0;
  double gps_edge_multiplier = 1000000.0;
  std::string kernel_type = "NONE";
  double kernel_delta = 0.1;
};

}  // namespace hdl_graph_slam

#endif