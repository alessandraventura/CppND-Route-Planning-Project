#include "route_planner.h"

#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (auto neighbor : current_node->neighbors) {
    neighbor->parent = current_node;
    neighbor->h_value = CalculateHValue(neighbor);
    neighbor->g_value =
        current_node->g_value + current_node->distance(*neighbor);
    open_list.emplace_back(neighbor);
    neighbor->visited = true;
  }
}

/**
 * Compare the F values of two cells.
 */
inline bool NodeCompare(RouteModel::Node *n1, RouteModel::Node *n2) {
  float f1 = n1->g_value + n1->h_value;  // f1 = g1 + h1
  float f2 = n2->g_value + n2->h_value;  // f2 = g2 + h2
  return f1 > f2;
}

RouteModel::Node *RoutePlanner::NextNode() {
  std::sort(this->open_list.begin(), this->open_list.end(), NodeCompare);
  RouteModel::Node *next_node;
  next_node = open_list.back();
  open_list.pop_back();
  return next_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(
    RouteModel::Node *current_node) {
  // Create path_found vector
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;
  std::vector<RouteModel::Node> path_sorted;

  RouteModel::Node *iterator_node = current_node;
  path_found.emplace_back(*current_node);
  while (iterator_node != start_node) {
    RouteModel::Node parent = *(iterator_node->parent);
    distance += iterator_node->distance(parent);
    iterator_node = iterator_node->parent;
    path_found.emplace_back(*iterator_node);
  }

  std::reverse(path_found.begin(), path_found.end());

  distance *= m_Model.MetricScale();  // Multiply the distance by the scale of
                                      // the map to get meters.
  return path_found;
}

void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = nullptr;
  std::vector<RouteModel::Node> final_path;

  // Preliminary operations: first step with the start node
  start_node->visited = true;
  open_list.emplace_back(start_node);
  AddNeighbors(start_node);

  while (open_list.size() > 0) {
    current_node = NextNode();
    if (current_node == end_node) {
      m_Model.path = ConstructFinalPath(current_node);
      return;
    } else {
      AddNeighbors(current_node);
    }
  }
}