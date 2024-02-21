#include "simple_mpc_publisher/simple_mpc_publisher.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

SimpleMpcPublisher::SimpleMpcPublisher() : Node("simple_mpc_publisher")
{
    float start_x, start_y, end_x, end_y, resolution;

    this->declare_parameter<float>("start_x",    0);
    this->declare_parameter<float>("start_y",    0);
    this->declare_parameter<float>("end_x",      0);
    this->declare_parameter<float>("end_y",      0);
    this->declare_parameter<float>("resolution", 0);

    this->get_parameter<float>("start_x",    start_x);
    this->get_parameter<float>("start_y",    start_y);
    this->get_parameter<float>("end_x",      end_x);
    this->get_parameter<float>("end_y",      end_y);
    this->get_parameter<float>("resolution", resolution);

    pub_trajectory_           = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("/planning/scenario_planning/trajectory", 1);
    pub_odometry_             = this->create_publisher<nav_msgs::msg::Odometry>("/localization/kinematic_state", 1);
    pub_accel_                = this->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>("/localization/acceleration", 1);
    pub_gear_                 = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", 1);
    pub_operation_mode_state_ = this->create_publisher<autoware_adapi_v1_msgs::msg::OperationModeState>("/system/operation_mode/state", rclcpp::QoS(1).transient_local().reliable());
    pub_trajectory_visualization_           = this->create_publisher<visualization_msgs::msg::MarkerArray>("/simple_mpc_trajectory", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    sub_gps_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/lexus3/gps/duro/current_pose", 10, std::bind(&SimpleMpcPublisher::gps_callback, this, std::placeholders::_1));
    sub_velocity_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status", 1, std::bind(&SimpleMpcPublisher::velocity_callback, this, std::placeholders::_1));
    sub_accel = this->create_subscription<pacmod3_msgs::msg::SystemRptFloat>(
        "/pacmod/accel_rpt", 1, std::bind(&SimpleMpcPublisher::accel_callback, this, std::placeholders::_1));

    marker_array.markers.clear();
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map_zala_0";
    marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    marker.lifetime = rclcpp::Duration(0, 0);
    marker.ns = "trajectory";
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1;

    trajectory.header.frame_id = "map";

    float fullDistance = sqrt(pow(end_x - start_x, 2) + pow(end_y - start_y, 2));

    for (float i = 0; i <= fullDistance; i+=resolution)
    {
        autoware_auto_planning_msgs::msg::TrajectoryPoint tp;
        // position
        tp.pose.position.x = start_x + i * (end_x - start_x) / fullDistance;
        tp.pose.position.y = start_y + i * (end_y - start_y) / fullDistance;
        tp.pose.position.z = 0;

        marker.points.push_back(tp.pose.position);

        // orientation
        float yaw = atan2(end_y - start_y, end_x - start_x);
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0, 0, yaw);
        quat_tf.normalize();
        tp.pose.orientation = tf2::toMsg(quat_tf);

        // speed
        tp.longitudinal_velocity_mps = 20 / 3.6;

        // tp.lateral_velocity_mps;
        // tp.acceleration_mps2;
        // time_from_start
        // heading_rate_rps
        // front_wheel_angle_rad
        // rear_wheel_angle_rad

        trajectory.points.push_back(tp);
    }

    marker_array.markers.push_back(marker);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33), std::bind(&SimpleMpcPublisher::timer_callback, this));
}

void SimpleMpcPublisher::timer_callback()
{
    // trajectory
    trajectory.header.stamp = rclcpp::Clock().now();
    pub_trajectory_->publish(trajectory);
    pub_trajectory_visualization_->publish(marker_array);

    // odometry
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = rclcpp::Clock().now();
    odometry.header.frame_id = "map";
    odometry.child_frame_id  = "base_link";
    odometry.pose.pose = gps_pose.pose;
    odometry.twist.twist.linear.x = velocity;
    pub_odometry_->publish(odometry);

    // acceleration
    geometry_msgs::msg::AccelWithCovarianceStamped accel;
    accel.header.stamp = rclcpp::Clock().now();
    accel.accel.accel.linear.x = currentAcceleration;
    pub_accel_->publish(accel);

    // gear
    autoware_auto_vehicle_msgs::msg::GearCommand gear;
    gear.stamp = rclcpp::Clock().now();
    gear.command = 2; // Drive
    pub_gear_->publish(gear);

    // operation mode
    autoware_adapi_v1_msgs::msg::OperationModeState operation_mode_state;
    operation_mode_state.stamp = rclcpp::Clock().now();
    operation_mode_state.mode = operation_mode_state.AUTONOMOUS;
    operation_mode_state.is_autoware_control_enabled = true;
    pub_operation_mode_state_->publish(operation_mode_state);
}

void SimpleMpcPublisher::velocity_callback(const std::shared_ptr<autoware_auto_vehicle_msgs::msg::VelocityReport> msg_)
{
    velocity = msg_->longitudinal_velocity;
}

void SimpleMpcPublisher::gps_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg_)
{
    gps_pose = *msg_;
}

void SimpleMpcPublisher::accel_callback(const std::shared_ptr<const pacmod3_msgs::msg::SystemRptFloat>& msg_)
{
    currentAcceleration = msg_->output;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleMpcPublisher>());
    rclcpp::shutdown();
    return 0;
}