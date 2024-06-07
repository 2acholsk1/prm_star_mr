/*
The MIT License (MIT)

Copyright (C) 2024 Some RISA Students - All Rights Reserved
You may use, distribute and modify this code under the
terms of the MIT license.
You should have received a copy of the MIT license with
this file.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the “Software”), to deal in the
Software without restriction, including without limitation the rights to use, copy,
modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
*/

#include "nav2_prmstar_planner/prmstar_planner.hpp"
#include "dubins.hpp"
namespace nav2_prmstar_planner
{

void prmstar::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".num_samples", rclcpp::ParameterValue(100));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".connection_radius", rclcpp::ParameterValue(1.0));

  node_->get_parameter(name_ + ".num_samples", num_samples_);
  node_->get_parameter(name_ + ".connection_radius", connection_radius_);
}

void prmstar::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "Cleaning up plugin %s of type prmstar",
    name_.c_str());
}

void prmstar::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type prmstar",
    name_.c_str());
}

void prmstar::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type prmstar",
    name_.c_str());
}

nav_msgs::msg::Path prmstar::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only accept start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only accept goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  // Generate PRM* roadmap
  std::vector<Node> roadmap = generateRoadmap(start, goal);
  
  // Find shortest path in the roadmap
  std::vector<Node*> path = findShortestPath(roadmap, start, goal);

  // Convert the path to ROS message format
  for (auto node : path)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = node->x;
    pose.pose.position.y = node->y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  return global_path;
}

std::vector<Node> prmstar::generateRoadmap(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  std::vector<Node> nodes;
  std::default_random_engine generator;
  // Pobierz granice obszaru mapy
  double map_min_x = costmap_->getOriginX();
  double map_max_x = map_min_x + costmap_->getSizeInMetersX();
  double map_min_y = costmap_->getOriginY();
  double map_max_y = map_min_y + costmap_->getSizeInMetersY();

  // Ustaw zakresy losowania na podstawie granic mapy
  std::uniform_real_distribution<double> distribution_x(map_min_x/4, map_max_x/4);
  std::uniform_real_distribution<double> distribution_y(map_min_y/4, map_max_y/4);

  // Add start and goal to the nodes
  nodes.push_back({start.pose.position.x, start.pose.position.y});
  nodes.push_back({goal.pose.position.x, goal.pose.position.y});

  // Sample random nodes
  for (int i = 0; i < num_samples_; ++i)
  {
    double x = distribution_x(generator);
    double y = distribution_y(generator);
    // Check if the sampled point is in collision
    if (!isInCollision(x, y)) {
      nodes.push_back({x, y});
    }
  }

  // Connect nodes within connection radius
  for (auto& node1 : nodes)
  {
    for (auto& node2 : nodes)
    {
      if (&node1 != &node2 && distance(node1, node2) < connection_radius_ && !collisionCheck(node1, node2))
      {
        node1.neighbors.push_back(&node2);
        node2.neighbors.push_back(&node1);
      }
    }
  }

  return nodes;
}

bool prmstar::isInCollision(double x, double y)
{
  // Check if the point (x, y) is in collision with the map
  unsigned int mx, my;
  if (costmap_->worldToMap(x, y, mx, my)) {
    return costmap_->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }
  return true; // Assume out of bounds points are in collision
}

bool prmstar::collisionCheck(const Node& a, const Node& b)
{
  // Define initial and goal configurations for Dubins path
  double q0[] = {a.x, a.y, 0}; // Initial configuration (x, y, theta)
  double q1[] = {b.x, b.y, 0}; // Final configuration (x, y, theta)
  double rho = 0.1; // Define a turning radius for the Dubins path

  HybridAStar::DubinsPath path;
  int ret = HybridAStar::dubins_init(q0, q1, rho, &path);
  if (ret != 0) {
    return true; // Path initialization failed, assume in collision
  }

  double stepSize = 0.01; // Define a step size for sampling the Dubins path
  for (double t = 0; t < HybridAStar::dubins_path_length(&path); t += stepSize) {
    double q[3];
    HybridAStar::dubins_path_sample(&path, t, q);
    if (isInCollision(q[0], q[1])) {
      return true; // A sampled point is in collision
    }
}
}

double prmstar::distance(const Node& a, const Node& b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double prmstar::heuristic(const Node& a, const Node& b)
{
  // Euclidean distance as the heuristic
  return distance(a, b);
}

std::vector<Node*> prmstar::findShortestPath(const std::vector<Node>& nodes, const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  Node* start_node = nullptr;
  Node* goal_node = nullptr;

  // Find the start and goal nodes
  for (const auto& node : nodes)
  {
    if (node.x == start.pose.position.x && node.y == start.pose.position.y)
    {
      start_node = const_cast<Node*>(&node);
    }
    if (node.x == goal.pose.position.x && node.y == goal.pose.position.y)
    {
      goal_node = const_cast<Node*>(&node);
    }
  }

  if (!start_node || !goal_node)
  {
    return {}; // Return empty path if start or goal node is not found
  }

  start_node->g = 0;
  start_node->f = heuristic(*start_node, *goal_node);

  std::vector<Node*> open_set = {start_node};

  while (!open_set.empty())
  {
    // Find the node with the lowest f score
    auto current_it = std::min_element(open_set.begin(), open_set.end(), [](Node* a, Node* b) {
      return a->f < b->f;
    });
    Node* current = *current_it;

    if (current == goal_node)
    {
      // Reconstruct the path
      std::vector<Node*> path;
      for (Node* node = goal_node; node != nullptr; node = node->parent)
      {
        path.push_back(node);
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    open_set.erase(current_it);

    for (Node* neighbor : current->neighbors)
    {
      double tentative_g = current->g + distance(*current, *neighbor);

      if (tentative_g < neighbor->g)
      {
        neighbor->parent = current;
        neighbor->g = tentative_g;
        neighbor->h = heuristic(*neighbor, *goal_node);
        neighbor->f = neighbor->g + neighbor->h;

        if (std::find(open_set.begin(), open_set.end(), neighbor) == open_set.end())
        {
          open_set.push_back(neighbor);
        }
      }
    }
  }

  return {}; // Return empty path if no path is found
}

}  // namespace nav2_prmstar_planner

PLUGINLIB_EXPORT_CLASS(nav2_prmstar_planner::prmstar, nav2_core::GlobalPlanner)
