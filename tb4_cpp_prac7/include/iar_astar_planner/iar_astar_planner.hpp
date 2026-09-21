#ifndef IAR_ASTAR_PLANNER__IAR_ASTAR_PLANNER_HPP_
#define IAR_ASTAR_PLANNER__IAR_ASTAR_PLANNER_HPP_

#include "iar_astar_planner/navfn.hpp"

#include "nav2_core/global_planner.hpp"

namespace iar_astar_planner
{

class IarAstarPlanner: public nav2_core::GlobalPlanner
{
public:
  IarAstarPlanner();
  ~IarAstarPlanner();

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, //shared pointer to parent node
    std::string name, // planner name
    std::shared_ptr<tf2_ros::Buffer> tf, //tf buffer pointer
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros //shared pointer to costmap
  ) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose) override; 

protected:
  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;
  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  nav2_util::LifecycleNode::SharedPtr parent_node_;
  // planner's name 
  std::string planner_name_;
  // The global frame of the costmap
  std::string global_frame_;

  // declares a logger instance and initialized with a specific name 
  // "IarAstarPlanner" to uniquely identify the logger
  rclcpp::Logger logger_{rclcpp::get_logger("IarAstarPlanner")}; 

  // Planner based on navigation function, methods in this class is 
  // used for path planning
  std::unique_ptr<NavFn> planner_;

  bool getPathFromPotential(nav_msgs::msg::Path & path);
};

}  // namespace iar_astar_planner

#endif  // IAR_ASTAR_PLANNER__IAR_ASTAR_PLANNER_HPP_
