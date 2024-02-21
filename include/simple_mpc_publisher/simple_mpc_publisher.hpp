#ifndef SIMPLE_MPC_PUBLISHER_HPP_
#define SIMPLE_MPC_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pacmod3_msgs/msg/system_rpt_float.hpp>



class SimpleMpcPublisher : public rclcpp::Node
{
public:
    SimpleMpcPublisher();

    void timer_callback();

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_gps_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr pub_gear_;
    rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr pub_accel_;
    rclcpp::Publisher<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr pub_operation_mode_state_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_trajectory_visualization_;
    rclcpp::Subscription<pacmod3_msgs::msg::SystemRptFloat>::SharedPtr               sub_accel;

    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr sub_velocity_;
    // TODO accel Subscription

    void velocity_callback(const std::shared_ptr<autoware_auto_vehicle_msgs::msg::VelocityReport> msg_);
    void gps_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg_);
    // acceleration callback
    void accel_callback(const std::shared_ptr<const pacmod3_msgs::msg::SystemRptFloat>& msg_);

    autoware_auto_planning_msgs::msg::Trajectory trajectory;
    float velocity;
    float currentAcceleration;
    geometry_msgs::msg::PoseStamped gps_pose;
    visualization_msgs::msg::MarkerArray marker_array;
};

#endif  // SIMPLE_MPC_PUBLISHER_HPP_