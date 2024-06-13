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
    node_, name_ + ".num_samples", rclcpp::ParameterValue(1000));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".gamma", rclcpp::ParameterValue(6.0));  // Updated gamma value

  node_->get_parameter(name_ + ".num_samples", num_samples_);
  node_->get_parameter(name_ + ".gamma", gamma_);
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
  // adjustOrientations(path);

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
  for (size_t i = 0; i < path.size() - 1; ++i) {
    Node* current = path[i];
    Node* next = path[i + 1];

    double angle = std::atan2(next->y - current->y, next->x - current->x);
    current->theta = angle;
  }

  // Adjust the orientation of the goal node to match the direction to the previous node
  if (path.size() > 1) {
    Node* last = path.back();
    Node* second_last = path[path.size() - 2];

    double angle = std::atan2(last->y - second_last->y, last->x - second_last->x);
    last->theta = angle;
  }
}

std::vector<Node> prmstar::generateRoadmap(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  std::vector<Node> nodes;
  std::default_random_engine generator;

  // Set random sampling range based on map boundaries
  std::uniform_real_distribution<double> distribution_x(-3.0, 3.0);
  std::uniform_real_distribution<double> distribution_y(-3.0, 3.0);
  std::uniform_real_distribution<double> distribution_yaw(-M_PI, M_PI);

  // Add start and goal to nodes
  nodes.push_back({start.pose.position.x, start.pose.position.y, tf2::getYaw(start.pose.orientation)});
  nodes.push_back({goal.pose.position.x, goal.pose.position.y, tf2::getYaw(goal.pose.orientation)});

  // Sample random nodes
  for (int i = 0; i < num_samples_; ++i)
  {
    double x = distribution_x(generator);
    double y = distribution_y(generator);
    double yaw = distribution_yaw(generator);

    // Check collision for sampled point
    if (!isInCollision(x, y)) {
      nodes.push_back({x, y, yaw});
    }
  }

  double connection_radius_temp = 1.0;

  double gamma_ = 1.0;
  int i = 2;

  // Connect nodes within connection radius
  for (auto& node1 : nodes)
  {
    for (auto& node2 : nodes)
    {
      if (&node1 != &node2 && distance(node1, node2) < connection_radius_temp && !collisionCheck(node1, node2))
      {


        node1.neighbors.push_back(&node2);
        node2.neighbors.push_back(&node1);
      }
    }

    connection_radius_temp = gamma_ * std::pow(std::log(i) / i, 1.0 / 2.0);

    i++;
  }

  return nodes;
}


std::vector<geometry_msgs::msg::PoseStamped> prmstar::generateDubinsPath(const Node& a, const Node& b)
{
  std::vector<geometry_msgs::msg::PoseStamped> dubins_path;
  double q0[] = {a.x, a.y, a.theta}; // Start configuration
  double q1[] = {b.x, b.y, b.theta}; // End configuration
  double turning_radius = 0.3; // Initial turning radius

  HybridAStar::DubinsPath path;
  int max_attempts = 10; // Maximum number of attempts to find a collision-free path
  bool path_found = false;

  for (int attempt = 0; attempt < max_attempts; ++attempt) {
    if (dubins_init(q0, q1, turning_radius, &path) == 0) {
      double max_points = 100; // Maximum number of points you want to generate
      double length = dubins_path_length(&path);

      double step_size = length / max_points; // Calculate step size based on path length

      // Ensure step_size is not zero to avoid division by zero
      if (step_size > length) {
        step_size = length;
      }

      path_found = true;
      for (double t = 0; t < length; t += step_size)
      {
        double q[3];
        if (dubins_path_sample(&path, t, q) == 0) {
          if (isInCollision(q[0], q[1])) {
            RCLCPP_WARN(node_->get_logger(), "Dubins path collides with obstacle. Trying larger turning radius.");
            path_found = false;
            break; // Exit the loop and try with a larger turning radius
          }
          geometry_msgs::msg::PoseStamped pose;
          pose.pose.position.x = q[0];
          pose.pose.position.y = q[1];
          tf2::Quaternion quat;
          quat.setRPY(0, 0, q[2]);
          pose.pose.orientation = tf2::toMsg(quat);
          dubins_path.push_back(pose);
        }
      }
    }
    if (path_found) {
      break; // Exit the loop if a valid path is found
    }
    turning_radius += 0.1; // Increase the turning radius and try again
    dubins_path.clear(); // Clear the previous path
  }

  if (!path_found) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to generate a collision-free Dubins path after multiple attempts.");
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
  // Euclidean distance as the heuristic
  double euclidean_distance = std::hypot(a.x - b.x, a.y - b.y);

  // Additional cost for proximity to obstacles
  double obstacle_proximity_cost = 0.0;
  double proximity_threshold = 0.2; // Threshold distance to consider obstacle proximity
  for (double dx = -proximity_threshold; dx <= proximity_threshold; dx += 0.1) {
    for (double dy = -proximity_threshold; dy <= proximity_threshold; dy += 0.1) {
      if (isInCollision(a.x + dx, a.y + dy)) {
        obstacle_proximity_cost += 0.5; // Increase cost if near obstacle
      }
    }
  }

  return euclidean_distance + obstacle_proximity_cost;
}

std::vector<Node*> prmstar::findShortestPath(const std::vector<Node>& nodes, const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal)
{
  Node* start_node = nullptr;
  Node* goal_node = nullptr;

  // Find the start and goal nodes
  for (auto& node : nodes)
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

        // Calculate orientation (theta) for the neighbor node
        neighbor->theta = atan2(neighbor->y - current->y, neighbor->x - current->x);

        if (std::find(open_set.begin(), open_set.end(), neighbor) == open_set.end())
        {
          open_set.push_back(neighbor);
        }
      }
    }
  }

  return {}; // Return empty path if no path is found
}


} // namespace nav2_prmstar_planner


PLUGINLIB_EXPORT_CLASS(nav2_prmstar_planner::prmstar, nav2_core::GlobalPlanner)