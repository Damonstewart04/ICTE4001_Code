#include "iar_astar_planner/iar_astar_planner.hpp"

namespace iar_astar_planner
{

IarAstarPlanner::IarAstarPlanner()
{
  tf_ = nullptr;
  costmap_ = nullptr;
}

IarAstarPlanner::~IarAstarPlanner()
{
  RCLCPP_INFO(
    logger_, "Destroying plugin %s of type IarAstarPlanner",
    planner_name_.c_str()
  );
}

void 
IarAstarPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, //shared pointer to parent node
    std::string name, // planner name
    std::shared_ptr<tf2_ros::Buffer> tf, //tf buffer pointer
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros //shared pointer to costmap
)
{
    tf_ = tf;
    planner_name_ = name;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    parent_node_ = parent.lock();
    logger_ = parent_node_->get_logger();

    RCLCPP_INFO(
        logger_, "Configuring plugin %s of type IarAstarPlanner", planner_name_.c_str()
    );

    // Create a planner based on the new costmap size
    planner_ = std::make_unique<NavFn>(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
}

void
IarAstarPlanner::activate()
{
    RCLCPP_INFO(
        logger_, "Activating plugin %s of type IarAstarPlanner", planner_name_.c_str()
    );
}

void
IarAstarPlanner::deactivate()
{
    RCLCPP_INFO(
        logger_, "Deactivating plugin %s of type IarAstarPlanner", planner_name_.c_str()
    );
}

void
IarAstarPlanner::cleanup()
{
    RCLCPP_INFO(
        logger_, "Cleaning up plugin %s of type IarAstarPlanner", planner_name_.c_str()
    );
    planner_.reset();
}


nav_msgs::msg::Path 
IarAstarPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose)
{
    nav_msgs::msg::Path path;

    unsigned int mx, my;
    // Check whether goal pose is valid or not
    if(!costmap_->worldToMap(goal_pose.pose.position.x, goal_pose.pose.position.y, mx, my))
    {
        RCLCPP_WARN(logger_, "The goal pose is off the global costmap. Planning will fail.");
        return path;
    }
    else
    {   
        // Check wether the goal pose is in an lethal obstacle cell
        if (costmap_->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE)
        {
            RCLCPP_WARN(logger_, "Failed to create a unique pose path because of obstacles");
            return path;
        }
    }
    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    // Check whether start pose is valid or not, we will not check whether the start pose is in
    // a lethal obstacle cell, as it is the current pose.
    if(!costmap_->worldToMap(start_pose.pose.position.x, start_pose.pose.position.y, mx, my))
    {
        RCLCPP_WARN(logger_, "The start pose is off the global costmap. Planning will fail.");
        return path;
    }

    int map_start[2];
    map_start[0] = mx;
    map_start[1] = my;


    path.header.stamp = parent_node_->now();
    path.header.frame_id = global_frame_;
    // clear the starting cell within the costmap because we know it can't be an obstacle
    costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);

    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    int nx, ny;
    nx = costmap_->getSizeInCellsX();
    ny = costmap_->getSizeInCellsY();

    // check the costmap whether is outdated, if it is outdated, update the costmap in planner
    if (!planner_.get() || planner_->nx_ != nx || planner_->ny_ != ny)
      planner_->setNavArr(nx, ny);

    planner_->setCostmap(costmap_->getCharMap());
    lock.unlock();

    planner_->setStart(map_start);
    planner_->setGoal(map_goal);
    if(planner_->setNavFn()) // check if navfn is setup properlly
    {
      // check if navfn can be computed to find a path
      if(planner_->propAstar(std::max(nx * ny / 20, nx + ny)))
      {
        if(!getPathFromPotential(path))
        {
              RCLCPP_ERROR(logger_, "Failed to create a plan from potential when a legal"
              " potential was found. This shouldn't happen.");
        }
        else
        {
          geometry_msgs::msg::PoseStamped goal = goal_pose;
          goal.header.stamp = parent_node_->now();
          path.poses.push_back(goal);
        }
      }
    }
    return path;
}

bool
IarAstarPlanner::getPathFromPotential(nav_msgs::msg::Path & path)
{
  path.poses.clear();
  const int & max_cycles = (costmap_->getSizeInCellsX() >= costmap_->getSizeInCellsY()) ?
    (costmap_->getSizeInCellsX() * 4) : (costmap_->getSizeInCellsY() * 4);
  int path_len = planner_->calcPath(max_cycles);
  if (path_len == 0) {
    return false;
  }

  for(int i = path_len - 1; i>=0; --i){
    double world_x, world_y;
    costmap_->mapToWorld(planner_->pathx_[i], planner_->pathy_[i], world_x, world_y);
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = parent_node_->now();
    pose.header.frame_id = global_frame_;
    path.poses.push_back(pose);
  }

  return !path.poses.empty();
}

}  // namespace iar_astar_planner


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(iar_astar_planner::IarAstarPlanner, nav2_core::GlobalPlanner)
