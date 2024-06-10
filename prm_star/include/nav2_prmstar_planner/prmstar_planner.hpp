// The MIT License (MIT)
//
// Copyright (C) 2024 Some RISA Students - All Rights Reserved
// You may use, distribute and modify this code under the
// terms of the MIT license.
// You should have received a copy of the MIT license with
// this file.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the “Software”), to deal in the
// Software without restriction, including without limitation the rights to use, copy,
// modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// */

#ifndef NAV2_prmstar_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define NAV2_prmstar_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <random>
#include <limits>

#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/global_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_prmstar_planner
{

struct Node
{
  double x, y; // Position
  std::vector<Node*> neighbors; // Connected nodes
  double g = std::numeric_limits<double>::infinity(); // Cost from start node
  double h = 0; // Heuristic cost to goal node
  double f = std::numeric_limits<double>::infinity(); // Total cost (g + h)
  Node* parent = nullptr;
};

class prmstar : public nav2_core::GlobalPlanner
{
public:
  prmstar() = default;
  ~prmstar() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  std::vector<Node> generateRoadmap(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal);
  double distance(const Node& a, const Node& b);
  double heuristic(const Node& a, const Node& b);
  std::vector<Node*> findShortestPath(const std::vector<Node>& nodes, const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal);
  bool isInCollision(double x, double y);
  bool collisionCheck(const Node& a, const Node& b);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D* costmap_;
  std::string global_frame_;
  int num_samples_;
  double connection_radius_;
};

}  // namespace nav2_prmstar_planner

#endif  // NAV2_PRMSTAR_PLANNER_HPP_
