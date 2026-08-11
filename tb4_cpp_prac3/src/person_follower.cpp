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
    this->declare_parameter<double>("following_distance", 0.5);
    this->declare_parameter<float>("following_angle", 0);
    this->declare_parameter<float>("angle_control_gain", 1.0);
    this->declare_parameter<float>("distance_control_gain", 0.5);

    // Get parameter values
    this->get_parameter("following_distance", following_distance_);
    this->get_parameter("following_angle", following_angle_);
    this->get_parameter("angle_control_gain", angle_control_gain_);
    this->get_parameter("distance_control_gain", distance_control_gain_);

    // Print parameter values
    RCLCPP_INFO(this->get_logger(), "following_distance: %.2f", following_distance_);
    RCLCPP_INFO(this->get_logger(), "following_angle: %.2f", following_angle_);
    RCLCPP_INFO(this->get_logger(), "angle_control_gain: %.2f", angle_control_gain_);
    RCLCPP_INFO(this->get_logger(), "distance_control_gain: %.2f", distance_control_gain_);

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
  double following_angle_;
  double angle_control_gain_;
  double distance_control_gain_;
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

  // Define Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

void PersonFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);

  // finds the minimum element in the ranges vector of the scan_msg. It returns an iterator
  // (min_distance) pointing to the smallest value in the range.
  // filter out values < 0.2m
  auto min_distance = std::min_element(scan_msg->ranges.begin(), scan_msg->ranges.end(), [](float a, float b)
                                       {
        bool a_valid = (a >= 0.4);
        bool b_valid = (b >= 0.4);
        if (a_valid && b_valid) return a < b;
        if (a_valid) return true;
        return false; });
  // Extracts the actual minimum value from the iterator obtained in the previous step.
  float min_value = *min_distance;
  // Calculates the index of the minimum value in the ranges vector by finding the distance
  // between the beginning of the vector and the iterator pointing to the minimum value.
  int min_index = std::distance(scan_msg->ranges.begin(), min_distance);
  // Calculate the angle corresponding to the index of the minimum value.
  float min_angle = (min_index - 320) * 2 * PI / 640.0;

  // calculate angle from lidar frame of reference
  // float l_min_angle = (scan_msg->angle_min) + scan_msg->angle_increment * min_index;
  // float r_min_angle = l_min_angle - PI / 2;


  geometry_msgs::msg::Twist cmd_vel_msg;

  if (min_value < 12)
  {
    // make movement if the object is within 12m
    cmd_vel_msg.angular.z = angle_control_gain_ * (min_angle - following_angle_);
    cmd_vel_msg.linear.x = 0.0;
    double travel_distance = distance_control_gain_ * (min_value - following_distance_);
    if (travel_distance >= 0.0) {
      cmd_vel_msg.linear.x = travel_distance;
    }
    
    RCLCPP_INFO(this->get_logger(), "Following object at distance %.2f with angle %.2f", min_value, min_angle);

  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "No Object is Detected");
    cmd_vel_msg.linear.x = 0.0;
  }

  cmd_vel_publisher_->publish(cmd_vel_msg);
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

      if (param_name == "following_angle")
      {
        following_angle_ = parameter.as_double();
      }

      if (param_name == "angle_control_gain")
      {
        angle_control_gain_ = parameter.as_double();
      }
      if (param_name == "distance_control_gain")
      {
        distance_control_gain_ = parameter.as_double();
      }
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
