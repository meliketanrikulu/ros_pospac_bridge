#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>  // Include PoseStamped message
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>
#include "ros_pospac_bridge/ros_pospac_bridge.hpp"
#include <Eigen/Geometry>
#include <fstream>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <memory>
#include <cmath>  // For M_PI and conversion functions

RosPospacBridge::RosPospacBridge() : Node("ros_pospac_bridge"){

    // Declare parameters
    bool start_gps_publisher = this->declare_parameter<bool>("start_gps_publisher", true);
    bool start_imu_publisher = this->declare_parameter<bool>("start_imu_publisher", true);
    bool start_pose_publisher = this->declare_parameter<bool>("start_pose_publisher", true);
    bool start_pose_array_publisher = this->declare_parameter<bool>("start_pose_array_publisher", true);
    bool start_twist_publisher = this->declare_parameter<bool>("start_twist_publisher", true);
    bool start_pose_stamped_publisher = this->declare_parameter<bool>("start_pose_stamped_publisher", true);

    // Initialize publishers based on parameters
    if (start_gps_publisher) {
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/ros_pospac_bridge/gps_fix", 10);
    }
    if (start_imu_publisher) {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/ros_pospac_bridge/imu_data", 10);
    }
    if (start_pose_publisher) {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ros_pospac_bridge/pose_with_covariance", 10);
    }
    if (start_pose_array_publisher) {
        pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/ros_pospac_bridge/pose_array", 10);
    }
    if (start_twist_publisher) {
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/ros_pospac_bridge/twist_data", 10);
    }
    if (start_pose_stamped_publisher) {
        pose_stamped_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ros_pospac_bridge/pose_stamped", 10);
    }

    // Declare other parameters
    file_path_ = this->declare_parameter<std::string>("gps_data_file", "/home/javadibrahimli/ros2_ws/route2.txt");
    double origin_latitude_ = this->declare_parameter<double>("origin_latitude", 0.0);
    double origin_longitude_ = this->declare_parameter<double>("origin_longitude", 0.0);
    double origin_altitude_ = this->declare_parameter<double>("origin_altitude", 0.0);
    lidar_to_gnss_transform_.x = this->declare_parameter<double>("lidar_to_gnss_transform.x", 0.0);
    lidar_to_gnss_transform_.y = this->declare_parameter<double>("lidar_to_gnss_transform.y", 0.0);
    lidar_to_gnss_transform_.z = this->declare_parameter<double>("lidar_to_gnss_transform.z", 0.0);
    lidar_to_gnss_transform_.roll = this->declare_parameter<double>("lidar_to_gnss_transform.roll", 0.0);
    lidar_to_gnss_transform_.pitch = this->declare_parameter<double>("lidar_to_gnss_transform.pitch", 0.0);
    lidar_to_gnss_transform_.yaw = this->declare_parameter<double>("lidar_to_gnss_transform.yaw", 0.0);
    lidar_to_base_link_transform_.x = this->declare_parameter<double>("lidar_to_base_link_transform.x", 0.0);
    lidar_to_base_link_transform_.y = this->declare_parameter<double>("lidar_to_base_link_transform.y", 0.0);
    lidar_to_base_link_transform_.z = this->declare_parameter<double>("lidar_to_base_link_transform.z", 0.0);
    lidar_to_base_link_transform_.roll = this->declare_parameter<double>("lidar_to_base_link_transform.roll", 0.0);
    lidar_to_base_link_transform_.pitch = this->declare_parameter<double>("lidar_to_base_link_transform.pitch", 0.0);
    lidar_to_base_link_transform_.yaw = this->declare_parameter<double>("lidar_to_base_link_transform.yaw", 0.0);

    int origin_zone_;
    bool origin_northp_;
    // Convert origin latitude and longitude to UTM
    GeographicLib::UTMUPS::Forward(origin_latitude_, origin_longitude_, origin_zone_, origin_northp_, origin_easting_, origin_northing_);
    initial_altitude_ = origin_altitude_;

    // Start publishing data
    publishGpsData();
}

void RosPospacBridge::publishGpsData()
{
    std::ifstream file(file_path_);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path_.c_str());
        return;
    }

    std::string line;

    while (std::getline(file, line) && rclcpp::ok())
    {
        std::istringstream iss(line);
        double time, distance, easting, northing, ortho_height, latitude, longitude, ellipsoid_height;
        double roll, pitch, heading, east_velocity, north_velocity, up_velocity;
        double x_angular_rate, y_angular_rate, z_angular_rate;
        double x_acceleration, y_acceleration, z_acceleration;
        double east_sd, north_sd, height_sd, roll_sd, pitch_sd, heading_sd;

        if (iss >> time >> distance >> easting >> northing >> ortho_height >> latitude >> longitude >> ellipsoid_height
                >> roll >> pitch >> heading >> east_velocity >> north_velocity >> up_velocity
                >> x_angular_rate >> y_angular_rate >> z_angular_rate
                >> x_acceleration >> y_acceleration >> z_acceleration
                >> east_sd >> north_sd >> height_sd >> roll_sd >> pitch_sd >> heading_sd)
        {

            rclcpp::Time sensor_time(static_cast<uint64_t>(time * 1e9), RCL_ROS_TIME);

            // Convert latitude and longitude to UTM
            int zone;
            bool northp;
            double utm_easting, utm_northing;
            GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_easting, utm_northing);

            double local_easting = utm_easting - origin_easting_;
            double local_northing = utm_northing - origin_northing_;
            double relative_altitude = ortho_height - initial_altitude_;

            // Create and publish GPS Fix message
            if (gps_pub_) {
                auto gps_msg = createGpsMessage(latitude, longitude, ellipsoid_height,
                                                east_sd, north_sd, height_sd, sensor_time);
                gps_pub_->publish(gps_msg);  // Publish GPS Fix message
            }

            // Create PoseWithCovarianceStamped from GPS data
            auto pose_msg = createPoseMessage(local_easting, local_northing, relative_altitude, roll, pitch, heading,
                                              east_sd, north_sd, height_sd, roll_sd, pitch_sd, heading_sd, sensor_time);

            // Publish the final pose in the base_link frame
            if (pose_pub_) {
                pose_pub_->publish(pose_msg);
            }

            // Convert PoseWithCovarianceStamped to PoseStamped and publish
            if (pose_stamped_pub_) {
                auto pose_stamped_msg = createPoseStampedMessage(pose_msg);
                pose_stamped_pub_->publish(pose_stamped_msg);
            }

            // Add the pose to the list of all poses
            all_poses_.push_back(poseWithCovarianceToPose(pose_msg));

            // Create and publish a PoseArray message
            if (pose_array_pub_) {
                geometry_msgs::msg::PoseArray pose_array_msg;
                pose_array_msg.header.stamp = pose_msg.header.stamp;
                pose_array_msg.header.frame_id = "base_link";
                pose_array_msg.poses = all_poses_;
                pose_array_pub_->publish(pose_array_msg);
            }

            // Publish IMU data
            if (imu_pub_) {
                auto imu_msg = createImuMessage(sensor_time, x_angular_rate, y_angular_rate, z_angular_rate,
                                                x_acceleration, y_acceleration, z_acceleration, roll, pitch, heading,
                                                roll_sd, pitch_sd, heading_sd);
                imu_pub_->publish(imu_msg);
            }

            // Publish Twist data
            if (twist_pub_) {
                publishTwistMessage(east_velocity, north_velocity, up_velocity,
                                    x_angular_rate, y_angular_rate, z_angular_rate, sensor_time);
            }
        }

        rclcpp::spin_some(this->get_node_base_interface());
    }
    file.close();
}

Eigen::Quaterniond RosPospacBridge::getQuaternionFromRPY(double roll, double pitch, double yaw)
{
    double roll_in_rad = roll * M_PI / 180.0;
    double pitch_in_rad = pitch * M_PI / 180.0;
    double yaw_in_rad = yaw * M_PI / 180.0;

    // convert ENU to NED
    Eigen::AngleAxisd angle_axis_x(roll_in_rad, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd angle_axis_y(pitch_in_rad, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd angle_axis_z(-yaw_in_rad + 1.5708, Eigen::Vector3d::UnitZ());

    Eigen::Matrix3d orientation_enu(angle_axis_z * angle_axis_y * angle_axis_x);

    Eigen::Quaterniond q(orientation_enu);

    return q;
}

sensor_msgs::msg::NavSatFix RosPospacBridge::createGpsMessage(double latitude, double longitude, double ellipsoid_height,
                                             double east_sd, double north_sd, double height_sd, rclcpp::Time timestamp)
{
    sensor_msgs::msg::NavSatFix gps_msg;
    gps_msg.header.stamp = timestamp;
    gps_msg.header.frame_id = "base_link"; // Changed to base_link
    gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    gps_msg.latitude = latitude;
    gps_msg.longitude = longitude;
    gps_msg.altitude = ellipsoid_height;

    gps_msg.position_covariance[0] = east_sd * east_sd;
    gps_msg.position_covariance[4] = north_sd * north_sd;
    gps_msg.position_covariance[8] = height_sd * height_sd;

    gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

    return gps_msg;
}

geometry_msgs::msg::PoseWithCovarianceStamped RosPospacBridge::createPoseMessage(double easting, double northing, double altitude,
                                                                double roll, double pitch, double yaw,
                                                                double east_sd, double north_sd, double height_sd,
                                                                double roll_sd, double pitch_sd, double yaw_sd, rclcpp::Time sensor_time_)
{
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp  = sensor_time_;
    pose_msg.header.frame_id = "base_link"; // Changed to base_link

    // Set position
    pose_msg.pose.pose.position.x = easting;
    pose_msg.pose.pose.position.y = northing;
    pose_msg.pose.pose.position.z = altitude;

    // Convert from Euler angles (assuming they are in degrees) to quaternion
    Eigen::Quaterniond q = getQuaternionFromRPY(roll, pitch, yaw);
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    // Set covariance (using the variances calculated from the standard deviations)
    pose_msg.pose.covariance[0] = east_sd * east_sd;    // Variance in X (easting)
    pose_msg.pose.covariance[7] = north_sd * north_sd;  // Variance in Y (northing)
    pose_msg.pose.covariance[14] = height_sd * height_sd;  // Variance in Z (altitude)

    pose_msg.pose.covariance[21] = roll_sd * roll_sd;   // Variance in roll
    pose_msg.pose.covariance[28] = pitch_sd * pitch_sd; // Variance in pitch
    pose_msg.pose.covariance[35] = yaw_sd * yaw_sd;     // Variance in yaw

    return pose_msg;
}

geometry_msgs::msg::Pose RosPospacBridge::poseWithCovarianceToPose(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_with_covariance)
{
    geometry_msgs::msg::Pose pose;
    pose.position = pose_with_covariance.pose.pose.position;
    pose.orientation = pose_with_covariance.pose.pose.orientation;
    return pose;
}

sensor_msgs::msg::Imu RosPospacBridge::createImuMessage(rclcpp::Time timestamp, double x_angular_rate, double y_angular_rate,
                                       double z_angular_rate, double x_acceleration, double y_acceleration,
                                       double z_acceleration, double roll, double pitch, double yaw,
                                       double roll_sd, double pitch_sd, double heading_sd)
{
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = timestamp;
    imu_msg.header.frame_id = "base_link"; // Changed to base_link

    Eigen::Quaterniond q = getQuaternionFromRPY(roll, pitch, yaw);  // Assuming angles are in degrees
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();

    imu_msg.angular_velocity.x = x_angular_rate;
    imu_msg.angular_velocity.y = y_angular_rate;
    imu_msg.angular_velocity.z = z_angular_rate;

    imu_msg.linear_acceleration.x = x_acceleration;
    imu_msg.linear_acceleration.y = y_acceleration;
    imu_msg.linear_acceleration.z = z_acceleration;

    imu_msg.orientation_covariance[0] = 1.0;
    imu_msg.orientation_covariance[4] = 1.0;
    imu_msg.orientation_covariance[8] = 1.0;

    imu_msg.angular_velocity_covariance[0] = 1.0;
    imu_msg.angular_velocity_covariance[4] = 1.0;
    imu_msg.angular_velocity_covariance[8] = 1.0;

    imu_msg.linear_acceleration_covariance[0] = 1.0;
    imu_msg.linear_acceleration_covariance[4] = 1.0;
    imu_msg.linear_acceleration_covariance[8] = 1.0;

    return imu_msg;
}

void RosPospacBridge::publishTwistMessage(double east_velocity, double north_velocity, double up_velocity,
                                          double x_angular_rate, double y_angular_rate, double z_angular_rate, rclcpp::Time sensor_time) {
    geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov_stamped_msg;

    twist_with_cov_stamped_msg.header.stamp = sensor_time;
    twist_with_cov_stamped_msg.header.frame_id = "base_link";  // Changed to base_link

    // Set linear velocities
    twist_with_cov_stamped_msg.twist.twist.linear.x = east_velocity;
    twist_with_cov_stamped_msg.twist.twist.linear.y = north_velocity;
    twist_with_cov_stamped_msg.twist.twist.linear.z = up_velocity;

    // Set angular velocities
    twist_with_cov_stamped_msg.twist.twist.angular.x = x_angular_rate;
    twist_with_cov_stamped_msg.twist.twist.angular.y = y_angular_rate;
    twist_with_cov_stamped_msg.twist.twist.angular.z = z_angular_rate;

    twist_with_cov_stamped_msg.twist.covariance[0] = 1.0;  // Variance in X (east velocity)
    twist_with_cov_stamped_msg.twist.covariance[7] = 1.0;  // Variance in Y (north velocity)
    twist_with_cov_stamped_msg.twist.covariance[14] = 1.0;  // Variance in Z (up velocity)

    // Publish the Twist message
    twist_pub_->publish(twist_with_cov_stamped_msg);
}

geometry_msgs::msg::PoseStamped RosPospacBridge::createPoseStampedMessage(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_with_covariance) {
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header = pose_with_covariance.header;
    pose_stamped_msg.pose = pose_with_covariance.pose.pose;
    return pose_stamped_msg;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosPospacBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
