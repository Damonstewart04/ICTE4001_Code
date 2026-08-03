#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_msgs/action/drive_arc.hpp"
#include "nav_msgs/msg/odometry.hpp"

class TB4ArcActionServer : public rclcpp::Node
{
public:
  using Drive_Arc = irobot_create_msgs::action::DriveArc;

  explicit TB4ArcActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("tb4_arc_action_server", options)
  {
    /*TODO  TASK - MILESTONE #2.2 Initialise the command velocity publisher share pointer*/
    this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::SystemDefaultsQoS());
    using namespace std::placeholders;
    /*TODO  TASK - MILESTONE #2.3 Initialise the odometry subsriber share pointer, and bing the call back function
      "odom_callback"
    */
    this->odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        rclcpp::SensorDataQoS(),
        std::bind(&TB4ArcActionServer::odom_callback, this, std::placeholders::_1));

    /*TODO  TASK - MILESTONE #2.4
      Initialsie the drive arc action server with name as "drive_arc_prac2", and bind call back functions for
      handling of accepting a goal, cancelling a action, and process the accepted goal
    */
    this->action_server_ = rclcpp_action::create_server<Drive_Arc>(
        this,
        "drive_arc_prac2",
        std::bind(&TB4ArcActionServer::handle_goal, this, _1, _2),
        std::bind(&TB4ArcActionServer::handle_cancel, this, _1),
        std::bind(&TB4ArcActionServer::handle_accepted, this, _1));
  }

private:
  /* TODO TASK - MILESTONE #2.1
  Define shared pointers for
    - action server for drive arc defined in irobot_create_msgs,
    - command velocity publisher
    - odometry subscriber
  */

  // Define drive distance server
  rclcpp_action::Server<Drive_Arc>::SharedPtr action_server_;
  // Define a command velocity publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  // Define drive distance subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  // odometry pointer
  nav_msgs::msg::Odometry::SharedPtr odom_;
  // odometry subscriber callback
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

  // Callback function for handling goals
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const irobot_create_msgs::action::DriveArc::Goal> goal);

  // Callback function for handling cancellation:
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle);

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle);

  // Action processing and update
  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle);
};

void TB4ArcActionServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  /*TODO TASK - MILESTONE #3.1
    save the odom_msg to the class member variable odom_
  */
  odom_ = odom_msg;
}

/*TODO TASK - MILESTONE #4.1
  complete the  call back function of "TB4ArcActionServer::handle_accepted" that handling the accepted goal
*/
void TB4ArcActionServer::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&TB4ArcActionServer::execute, this, _1), goal_handle}.detach();
}
/* TODO TASK - MILESTONE #4.2
  complete the  call back function of "TB4ArcActionServer::handle_cancel" that cancel the goal
  */
rclcpp_action::CancelResponse TB4ArcActionServer::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
/*TODO TASK - MILESTONE #4.3
  complete the  call back function of "TB4ArcActionServer::handle_goal" that accept goal,
  you should also print the goal details in the terminal
*/
rclcpp_action::GoalResponse TB4ArcActionServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const irobot_create_msgs::action::DriveArc::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(),
              "Received goal request with travel arc of %f m and maximum speed at %f m/s",
              goal->radius,
              goal->max_translation_speed);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
/* TODO TASKS - MILESTONE #5.1 ~ #5.3
  complete the  thread function "execute" to proccess the goal in the action request
*/
void TB4ArcActionServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Drive_Arc::Feedback>();
  auto result = std::make_shared<Drive_Arc::Result>();

  geometry_msgs::msg::Twist cmd_vel;
  int pub_freq = 100;
  rclcpp::Rate loop_rate(pub_freq);

  auto &remaining_travel_angle = feedback->remaining_angle_travel;

  double omega = 0.0;
  double linear_v = 0.0;
  double speed = goal->max_translation_speed;

  // calc kinematics
  if (goal->radius == 0.0)
  {
    omega = speed;
    linear_v = 0.0;
  }
  else
  {
    omega = speed / goal->radius;
    linear_v = speed;
  }

  // calc proper direction
  if (goal->angle < 0)
  {
    omega = -omega;
  }
  if (goal->translate_direction < 0)
  {
    linear_v = -linear_v;
  }
  cmd_vel.linear.set__x(goal->max_translation_speed);
  cmd_vel.angular.set__z(omega);

  int count = pub_freq * goal->angle / omega;

  geometry_msgs::msg::PoseStamped pose_stamped;

  for (int i = 0; (i < count) && rclcpp::ok(); ++i)
  {
    pose_stamped.header = odom_->header;
    pose_stamped.pose = odom_->pose.pose;
    if (goal_handle->is_canceling())
    {
      result->set__pose(pose_stamped);
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    // TODO: calculate remaining_arc_distance
    remaining_travel_angle = goal->angle - omega * i / pub_freq;
    // Publish the command velocity
    cmd_vel_publisher_->publish(cmd_vel);
    // Publish feedback
    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }
  // Check if goal is done
  if (rclcpp::ok())
  {
    result->set__pose(pose_stamped);
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TB4ArcActionServer>());
  rclcpp::shutdown();
  return 0;
}