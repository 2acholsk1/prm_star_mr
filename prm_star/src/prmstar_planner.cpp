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
    node_, name_ + ".num_samples", rclcpp::ParameterValue(5000));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".connection_radius", rclcpp::ParameterValue(1.5));

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

  // Adjust orientations to allow Dubins path generation
  adjustOrientations(path);

  // Convert the path to ROS message format and add PRM path
  for (size_t i = 0; i < path.size() - 1; ++i)
  {
    // Generate Dubins path between consecutive PRM nodes and add it to global path
    std::vector<geometry_msgs::msg::PoseStamped> dubins_path = generateDubinsPath(*path[i], *path[i + 1]);
    global_path.poses.insert(global_path.poses.end(), dubins_path.begin(), dubins_path.end());
  }

  return global_path;
}

void prmstar::adjustOrientations(std::vector<Node*>& path)
{
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(-1.5, 1.5); // Małe losowe przesunięcie

  for (size_t i = 0; i < path.size() - 1; ++i) {
    Node* current = path[i];
    Node* next = path[i + 1];

    double angle = std::atan2(next->y - current->y, next->x - current->x);
    angle += distribution(generator); // Dodanie małego losowego przesunięcia
    current->theta = angle;
  }

  // Adjust the orientation of the goal node to match the direction to the previous node
  if (path.size() > 1) {
    Node* last = path.back();
    Node* second_last = path[path.size() - 2];

    double angle = std::atan2(last->y - second_last->y, last->x - second_last->x);
    angle += distribution(generator); // Dodanie małego losowego przesunięcia
    last->theta = angle;
  }
}


std::vector<Node> prmstar::generateRoadmap(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  std::vector<Node> nodes;
  std::default_random_engine generator;
  // Get the map bounds
  double map_min_x = costmap_->getOriginX();
  double map_max_x = map_min_x + costmap_->getSizeInMetersX();
  double map_min_y = costmap_->getOriginY();
  double map_max_y = map_min_y + costmap_->getSizeInMetersY();

  // Set sampling ranges based on map bounds
  std::uniform_real_distribution<double> distribution_x(map_min_x, map_max_x);
  std::uniform_real_distribution<double> distribution_y(map_min_y, map_max_y);
  std::uniform_real_distribution<double> distribution_theta(-M_PI, M_PI);

  // Add start and goal to the nodes
  nodes.push_back({start.pose.position.x, start.pose.position.y, tf2::getYaw(start.pose.orientation)});
  nodes.push_back({goal.pose.position.x, goal.pose.position.y, tf2::getYaw(goal.pose.orientation)});

  // Sample random nodes
  for (int i = 0; i < num_samples_; ++i)
  {
    double x = distribution_x(generator);
    double y = distribution_y(generator);
    double theta = distribution_theta(generator);
    // Check if the sampled point is in collision
    if (!isInCollision(x, y)) {
      nodes.push_back({x, y, theta});
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

std::vector<geometry_msgs::msg::PoseStamped> prmstar::generateDubinsPath(const Node& a, const Node& b)
{
  std::vector<geometry_msgs::msg::PoseStamped> dubins_path;
  double q0[] = {a.x, a.y, a.theta}; // Start configuration
  double q1[] = {b.x, b.y, b.theta}; // End configuration
  double turning_radius = 0.4; // Define turning radius

  HybridAStar::DubinsPath path;
  if (dubins_init(q0, q1, turning_radius, &path) != 0) {
    RCLCPP_ERROR(node_->get_logger(), "Error generating Dubins path");
    return dubins_path;
  }

  double max_points = 100; // Maximum number of points you want to generate
  double length = dubins_path_length(&path);

  double step_size = length / max_points; // Calculate step size based on path length

  // Ensure step_size is not zero to avoid division by zero
  if (step_size > length) {
    step_size = length;
  }

  for (double t = 0; t < length; t += step_size)
  {
    double q[3];
    if (dubins_path_sample(&path, t, q) == 0) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = q[0];
      pose.pose.position.y = q[1];
      tf2::Quaternion quat;
      quat.setRPY(0, 0, q[2]);
      pose.pose.orientation = tf2::toMsg(quat);
      dubins_path.push_back(pose);
    }
  }

  return dubins_path;
}

bool prmstar::isInCollision(double x, double y)
{
  // Check if the point (x, y) is in collision with the map
  unsigned int mx, my;
  if (costmap_->worldToMap(x, y, mx, my)) {
    return costmap_->getCost(mx, my) >= 240;
  }
  return true; // Assume out of bounds points are in collision
}

bool prmstar::collisionCheck(const Node& a, const Node& b)
{
  std::vector<geometry_msgs::msg::PoseStamped> path = generateDubinsPath(a, b);
  for (const auto& pose : path)
  {
    if (isInCollision(pose.pose.position.x, pose.pose.position.y))
    {
      return true;
    }
  }
  return false;
}

double prmstar::distance(const Node& a, const Node& b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double prmstar::heuristic(const Node& a, const Node& b)
{
  return distance(a, b);
}

std::vector<Node*> prmstar::findShortestPath(const std::vector<Node>& roadmap, const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal)
{
  std::vector<Node*> path;

  // Create a priority queue for Dijkstra's algorithm
  std::priority_queue<std::pair<double, Node*>, std::vector<std::pair<double, Node*>>, std::greater<std::pair<double, Node*>>> pq;

  // Maps to store distances and predecessors
  std::unordered_map<Node*, double> distances;
  std::unordered_map<Node*, Node*> predecessors;

  // Initialize distances to infinity and start node distance to 0
  for (const auto& node : roadmap) {
    distances[const_cast<Node*>(&node)] = std::numeric_limits<double>::infinity();
  }

  Node* start_node = const_cast<Node*>(&roadmap[0]);
  distances[start_node] = 0.0;
  pq.push({0.0, start_node});

  // Dijkstra's algorithm
  while (!pq.empty())
  {
    Node* current = pq.top().second;
    pq.pop();

    if (current->x == goal.pose.position.x && current->y == goal.pose.position.y) {
      break; // Reached goal
    }

    for (Node* neighbor : current->neighbors)
    {
      double new_cost = distances[current] + distance(*current, *neighbor);
      if (new_cost < distances[neighbor]) {
        distances[neighbor] = new_cost;
        predecessors[neighbor] = current;
        pq.push({new_cost, neighbor});
      }
    }
  }

  // Reconstruct the path from goal to start
  Node* current = nullptr;
  for (auto& node : roadmap) {
    if (node.x == goal.pose.position.x && node.y == goal.pose.position.y) {
      current = const_cast<Node*>(&node);
      break;
    }
  }

  while (current != nullptr) {
    path.push_back(current);
    current = predecessors[current];
  }

  std::reverse(path.begin(), path.end());
  return path;
}

} // namespace nav2_prmstar_planner


PLUGINLIB_EXPORT_CLASS(nav2_prmstar_planner::prmstar, nav2_core::GlobalPlanner)