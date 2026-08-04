#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#define PI 3.14159265358
using rcl_interfaces::msg::ParameterType;

class PersonFollower : public rclcpp::Node
{
public:
  PersonFollower() : Node("person_follower")
  {
    /*TODO TASK 2 - MILESTONE #1.2
    1. Declare all parameters used for configuring the "following distance", "following angle", and all control gains. Their default values should be given as well.
    2. Get all parameter values from the constructor, and save them to private class element variables.
    3. Print all parameter values here.
    */

    // Declare parameters
    this->declare_parameter<double>("following_distance", 1.0);

    // Get parameter values
    this->get_parameter("following_distance", following_distance_);

    // Print parameter values
    RCLCPP_INFO(this->get_logger(), "following_distance: %.2f", following_distance_);

    //  Initalise the dynamic parameter handler
    dyn_params_handler_ = this->add_on_set_parameters_callback(
        std::bind(
            &PersonFollower::dynamicParametersCallback,
            this, std::placeholders::_1));

    // Publisher for the topic /cmd_vel
    this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::SystemDefaultsQoS());
    using namespace std::placeholders;
    // Subsriber to the /scan topic
    this->scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&PersonFollower::scan_callback, this, _1));
  }

private:
  // Define a command velocity publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  // Define a laser scan subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  // laser scan topic message pointer
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  std::recursive_mutex mutex_;

  /* TODO TASK 1 - MILESTONE #1.1
    Define all private element variables to store parameters.
  */
  double following_distance_;

  // Define Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
};

void PersonFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);
  /*TODO TASKS

    MILESTONE #3.1 - Process the received scan_msg to get the location of the closest object in robot's environment.
      NOTE: the four pillars of will be visible from the Lidar sensor, you have to remove the distance
      measurements of these four pillars by ignoring any measurement less than 0.2 meter.

    MILESTONE #3.2. You have to calculate the bearing and the range of the closest object with respect to the robot frame. You have
    to check the LaserScan message definition, and how the Lidar sensor is mounted with respective to
    the robot's coordinate.

    MILESTONE #3.3. Write a Person Follow Reactive Control that takes the bearing and range information of the closest
    object in the environment as the input and publish a message on topic /cmd_vel to control the motion of
    the robot.
  */
}

rcl_interfaces::msg::SetParametersResult
PersonFollower::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters)
  {
    const auto &param_type = parameter.get_type();
    const auto &param_name = parameter.get_name();
    if (param_type == ParameterType::PARAMETER_DOUBLE)
    {
      if (param_name == "following_distance")
      {
        following_distance_ = parameter.as_double();
        if (following_distance_ < 0.0)
        {
          RCLCPP_WARN(this->get_logger(), "You've set following_distance to be negative,"
                                          " this isn't allowed, so the alpha1 will be set to be zero.");
          following_distance_ = 0.0;
        }
      }
      /*
      TODO TASK 3 - MILESTONE # 2.1
      Check whether other parameters should be updated and if yes,
      store the updated value to the class variables defined in TASK 1 (Milestone # 1.1)
      */
    }
  }
  result.successful = true;
  return result;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PersonFollower>());
  rclcpp::shutdown();
  return 0;
}
