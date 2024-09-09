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

namespace ros_pospac_bridge {
// RosPospacBridge::RosPospacBridge() : Node("ros_pospac_bridge"){
RosPospacBridge::RosPospacBridge(const rclcpp::NodeOptions & node_options)
: Node("PoseCovarianceModifierNode", node_options)
{
    // Declare parameters
    bool enable_nav_sat_fix = this->declare_parameter<bool>("publishers.nav_sat_fix.enable", true);
    bool enable_imu = this->declare_parameter<bool>("publishers.imu.enable", true);
    bool enable_pose_with_cov = this->declare_parameter<bool>("publishers.pose_with_covariance_stamped.enable", true);
    bool enable_pose_array = this->declare_parameter<bool>("publishers.pose_array.enable", true);
    bool enable_twist_with_cov = this->declare_parameter<bool>("publishers.twist_with_covariance_stamped.enable", true);
    bool enable_pose = this->declare_parameter<bool>("publishers.pose_stamped.enable", true);
    bool enable_tf = this->declare_parameter<bool>("publishers.tf.enable", true);

    // Initialize publishers based on parameters
    if (enable_nav_sat_fix) {
        std::string nav_sat_fix_topic_name = this->declare_parameter<std::string>("publishers.nav_sat_fix.topic", "");
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(nav_sat_fix_topic_name, 10);
    }
    if (enable_imu) {
        std::string imu_topic_name = this->declare_parameter<std::string>("publishers.imu.topic", "");
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_name, 10);
    }
    if (enable_pose_with_cov) {
        std::string pose_with_cov_topic_name = this->declare_parameter<std::string>("publishers.pose_with_covariance_stamped.topic", "");
        pose_with_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_with_cov_topic_name, 10);
    }
    if (enable_pose_array) {
        std::string pose_array_topic_name = this->declare_parameter<std::string>("publishers.pose_array.topic", "");
        pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(pose_array_topic_name, 10);
    }
    if (enable_twist_with_cov) {
        std::string twist_with_cov_topic_name = this->declare_parameter<std::string>("publishers.twist_with_covariance_stamped.topic", "");
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(twist_with_cov_topic_name, 10);
    }
    if (enable_pose) {
        std::string pose_topic_name = this->declare_parameter<std::string>("publishers.pose_stamped.topic", "");
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_name, 10);
    }
    if (enable_tf) {
        tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(
        std::shared_ptr<rclcpp::Node>(this, [](auto) {}));
    }


    lidar_to_gnss_transform_ = declare_transform_parameters(this, "calibration.lidar_to_gnss");
    base_link_to_lidar_transform_ = declare_transform_parameters(this, "calibration.base_link_to_lidar");
    gnss_to_base_link_transform_ = declare_transform_parameters(this, "calibration.gnss_to_base_link");


    // Start publishing data
    publishGpsData();
}

void RosPospacBridge::publishGpsData()
{
    // Declare other parameters
    file_path_ = this->declare_parameter<std::string>("gps_data_file", "");

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

            rclcpp::Time sensor_time(static_cast<uint64_t>((time + 1723766400) * 1e9), RCL_ROS_TIME);



            // Create and publish GPS Fix message
            if (gps_pub_) {
                auto gps_msg = create_nav_sat_fix_msg(latitude, longitude, ellipsoid_height,
                                                east_sd, north_sd, height_sd, sensor_time);
                gps_pub_->publish(gps_msg);  // Publish GPS Fix message
            }



            ///////////////////////////////////////////////
            ///
            geometry_msgs::msg::Pose pose_msg = create_pose_msg(easting, northing, ortho_height, roll, pitch, heading);
            geometry_msgs::msg::Pose pose_msg_in_base_link = transform_pose_to_base_link(pose_msg);

            // geometry_msgs::msg::Pose pose_msg_in_base_link = pose_msg;

            if (pose_pub_) {
                geometry_msgs::msg::PoseStamped pose_stamped_msg = create_pose_stamped_msg(pose_msg_in_base_link, sensor_time);
                pose_pub_->publish(pose_stamped_msg);
            }

            // Publish the final pose in the base_link frame
            if (pose_with_cov_pub_) {
                geometry_msgs::msg::PoseWithCovarianceStamped pose_with_cov_msg = create_pose_with_cov_msg(pose_msg_in_base_link, east_sd, north_sd, height_sd, roll_sd, pitch_sd, heading_sd, sensor_time);
                pose_with_cov_pub_->publish(pose_with_cov_msg);
            }

            // Create and publish a PoseArray message
            if (pose_array_pub_) {
                // Add the pose to the list of all poses
                all_poses_.push_back(pose_msg_in_base_link);
                geometry_msgs::msg::PoseArray pose_array_msg;
                pose_array_msg.header.stamp = sensor_time;
                pose_array_msg.header.frame_id = "map";
                pose_array_msg.poses = all_poses_;
                pose_array_pub_->publish(pose_array_msg);
            }

            // Publish IMU data
            if (imu_pub_) {
                auto imu_msg = create_imu_msg(sensor_time, x_angular_rate, y_angular_rate, z_angular_rate,
                                                x_acceleration, y_acceleration, z_acceleration, roll, pitch, heading,
                                                roll_sd, pitch_sd, heading_sd);
                imu_pub_->publish(imu_msg);
            }

            // Publish Twist data
            if (twist_pub_) {
                geometry_msgs::msg::TwistWithCovarianceStamped twist_with_covariance_stamped_msg =  create_twist_msg(east_velocity, north_velocity, up_velocity,
                                                                                                                        x_angular_rate, y_angular_rate, z_angular_rate, sensor_time);
                twist_pub_->publish(twist_with_covariance_stamped_msg);
            }
            if (tf_br_) {
                geometry_msgs::msg::TransformStamped transform_stamped;
                transform_stamped.header.stamp = sensor_time;
                transform_stamped.header.frame_id = "map";
                transform_stamped.child_frame_id = "gnss_ins";
                transform_stamped.transform.translation.x = pose_msg_in_base_link.position.x;
                transform_stamped.transform.translation.y = pose_msg_in_base_link.position.y;
                transform_stamped.transform.translation.z = pose_msg_in_base_link.position.z;
                transform_stamped.transform.rotation = pose_msg_in_base_link.orientation;
                tf_br_->sendTransform(transform_stamped);
            }
        }

        rclcpp::spin_some(this->get_node_base_interface());
    }
    file.close();
}

geometry_msgs::msg::Pose RosPospacBridge::transform_pose_to_base_link(geometry_msgs::msg::Pose& gnss_pose){

    std::cout<<"gnss_pose x :"<<gnss_pose.position.x<<std::endl;
    std::cout<<"gnss_pose y :"<<gnss_pose.position.y<<std::endl;
    std::cout<<"gnss_pose z :"<<gnss_pose.position.z<<std::endl;
    std::cout<<"gnss_pose ori_x :"<<gnss_pose.orientation.x<<std::endl;
    std::cout<<"gnss_pose ori_y :"<<gnss_pose.orientation.y<<std::endl;
    std::cout<<"gnss_pose ori_z :"<<gnss_pose.orientation.z<<std::endl;
    std::cout<<"gnss_pose ori_w :"<<gnss_pose.orientation.w<<std::endl;

    tf2::Transform tf_map2gnss_transform{};
    tf2::fromMsg(gnss_pose, tf_map2gnss_transform);

    tf2::Transform tf_map2base_link{};
    tf_map2base_link = tf_map2gnss_transform * gnss_to_base_link_transform_;

    geometry_msgs::msg::Pose pose_in_base_link;
    tf2::toMsg(tf_map2base_link, pose_in_base_link);


    std::cout<<"pose_in_base_link x :"<<pose_in_base_link.position.x<<std::endl;
    std::cout<<"pose_in_base_link y :"<<pose_in_base_link.position.y<<std::endl;
    std::cout<<"pose_in_base_link z :"<<pose_in_base_link.position.z<<std::endl;
    std::cout<<"pose_in_base_link ori_x :"<<pose_in_base_link.orientation.x<<std::endl;
    std::cout<<"pose_in_base_link ori_y :"<<pose_in_base_link.orientation.y<<std::endl;
    std::cout<<"pose_in_base_link ori_z :"<<pose_in_base_link.orientation.z<<std::endl;
    std::cout<<"pose_in_base_link ori_w :"<<pose_in_base_link.orientation.w<<std::endl;

    return pose_in_base_link;
}

tf2::Transform RosPospacBridge::declare_transform_parameters(rclcpp::Node* node, const std::string& prefix) {


    double x = node->declare_parameter<double>(prefix + ".x", 0.0);
    double y = node->declare_parameter<double>(prefix + ".y", 0.0);
    double z = node->declare_parameter<double>(prefix + ".z", 0.0);
    double roll = node->declare_parameter<double>(prefix + ".roll", 0.0);
    double pitch = node->declare_parameter<double>(prefix + ".pitch", 0.0);
    double yaw = node->declare_parameter<double>(prefix + ".yaw", 0.0);
    // Pozisyon vektörünü oluştur
    tf2::Vector3 origin(x, y, z);

    // Euler açılarını (roll, pitch, yaw) kullanarak bir quaternion oluştur
    tf2::Quaternion rotation;
    rotation.setRPY(roll, pitch, yaw);

    // Transform objesini oluştur ve yukarıdaki vektör ve quaternion ile initialize et
    tf2::Transform transform;
    transform.setOrigin(origin);
    transform.setRotation(rotation);


    return transform;
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

sensor_msgs::msg::NavSatFix RosPospacBridge::create_nav_sat_fix_msg(double latitude, double longitude, double ellipsoid_height,
                                             double east_sd, double north_sd, double height_sd, rclcpp::Time timestamp)
{
    sensor_msgs::msg::NavSatFix gps_msg;
    gps_msg.header.stamp = timestamp;
    gps_msg.header.frame_id = "map";
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
geometry_msgs::msg::Pose RosPospacBridge::create_pose_msg(double easting, double northing, double altitude,
                                                            double roll, double pitch, double yaw) {
    // convert global UTM to MGRS
    GNSSStat gnss_stat_utm;

    gnss_stat_utm.x = easting;
    gnss_stat_utm.y = northing;
    gnss_stat_utm.z = altitude;
    gnss_stat_utm.coordinate_system = CoordinateSystem::UTM;
    gnss_stat_utm.zone = 35;
    gnss_stat_utm.northup = true;

    // convert latitude and longitude from UTM
    GeographicLib::UTMUPS::Reverse(gnss_stat_utm.zone,gnss_stat_utm.northup, gnss_stat_utm.x, gnss_stat_utm.y, gnss_stat_utm.latitude, gnss_stat_utm.longitude);

    gnss_stat_utm.altitude = gnss_stat_utm.z;

    GNSSStat gnss_stat_mgrs;
    const rclcpp::Logger & logger = this->get_logger();
    gnss_stat_mgrs = convertUTM2MGRS(gnss_stat_utm,MGRSPrecision::_100MICRO_METER,logger);

    // create posewithcov msg
    geometry_msgs::msg::Pose pose_msg;

    // Set position
    pose_msg.position.x = gnss_stat_mgrs.x;
    pose_msg.position.y = gnss_stat_mgrs.y;
    pose_msg.position.z = altitude;

    // Convert from Euler angles (assuming they are in degrees) to quaternion
    Eigen::Quaterniond q = getQuaternionFromRPY(roll, pitch, yaw);
    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();

    return pose_msg;

}

geometry_msgs::msg::PoseStamped RosPospacBridge::create_pose_stamped_msg(geometry_msgs::msg::Pose pose, rclcpp::Time timestamp) {

    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header.stamp = timestamp;
    pose_stamped_msg.header.frame_id = "map";
    pose_stamped_msg.pose = pose;
    return pose_stamped_msg;
}

geometry_msgs::msg::PoseWithCovarianceStamped RosPospacBridge::create_pose_with_cov_msg(geometry_msgs::msg::Pose pose,
                                                                double east_sd, double north_sd, double height_sd,
                                                                double roll_sd, double pitch_sd, double yaw_sd, rclcpp::Time sensor_time_)
{
    geometry_msgs::msg::PoseWithCovarianceStamped pose_with_cov_msg;
    pose_with_cov_msg.header.stamp = sensor_time_;
    pose_with_cov_msg.header.frame_id = "map";
    pose_with_cov_msg.pose.pose = pose;

    // Set covariance (using the variances calculated from the standard deviations)
    pose_with_cov_msg.pose.covariance[0] = east_sd * east_sd;    // Variance in X (easting)
    pose_with_cov_msg.pose.covariance[7] = north_sd * north_sd;  // Variance in Y (northing)
    pose_with_cov_msg.pose.covariance[14] = height_sd * height_sd;  // Variance in Z (altitude)

    pose_with_cov_msg.pose.covariance[21] = roll_sd * roll_sd;   // Variance in roll
    pose_with_cov_msg.pose.covariance[28] = pitch_sd * pitch_sd; // Variance in pitch
    pose_with_cov_msg.pose.covariance[35] = yaw_sd * yaw_sd;     // Variance in yaw

    return pose_with_cov_msg;
}

sensor_msgs::msg::Imu RosPospacBridge::create_imu_msg(rclcpp::Time timestamp, double x_angular_rate, double y_angular_rate,
                                       double z_angular_rate, double x_acceleration, double y_acceleration,
                                       double z_acceleration, double roll, double pitch, double yaw,
                                       double roll_sd, double pitch_sd, double heading_sd)
{
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = timestamp;
    imu_msg.header.frame_id = "gnss_ins";

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

    imu_msg.orientation_covariance[0] = roll_sd * roll_sd;
    imu_msg.orientation_covariance[4] = pitch_sd * pitch_sd;
    imu_msg.orientation_covariance[8] = heading_sd * heading_sd;

    imu_msg.angular_velocity_covariance[0] = 1.0;
    imu_msg.angular_velocity_covariance[4] = 1.0;
    imu_msg.angular_velocity_covariance[8] = 1.0;

    imu_msg.linear_acceleration_covariance[0] = 1.0;
    imu_msg.linear_acceleration_covariance[4] = 1.0;
    imu_msg.linear_acceleration_covariance[8] = 1.0;

    return imu_msg;
}

geometry_msgs::msg::TwistWithCovarianceStamped RosPospacBridge::create_twist_msg(double east_velocity, double north_velocity, double up_velocity,
                                          double x_angular_rate, double y_angular_rate, double z_angular_rate, rclcpp::Time sensor_time) {
    geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov_stamped_msg;

    twist_with_cov_stamped_msg.header.stamp = sensor_time;
    twist_with_cov_stamped_msg.header.frame_id = "gnss_ins";

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

    return twist_with_cov_stamped_msg;
}


RosPospacBridge::GNSSStat RosPospacBridge::convertUTM2MGRS(RosPospacBridge::GNSSStat gnss_stat_utm, const RosPospacBridge::MGRSPrecision precision, const rclcpp::Logger &logger){

    constexpr int GZD_ID_size = 5;  // size of header like "53SPU"

    RosPospacBridge::GNSSStat mgrs = gnss_stat_utm;
    mgrs.coordinate_system = RosPospacBridge::CoordinateSystem::MGRS;
    try {
        std::string mgrs_code;
        GeographicLib::MGRS::Forward(
                gnss_stat_utm.zone, gnss_stat_utm.northup, gnss_stat_utm.x, gnss_stat_utm.y, gnss_stat_utm.latitude, static_cast<int>(precision), mgrs_code);
        mgrs.mgrs_zone = std::string(mgrs_code.substr(0, GZD_ID_size));
        mgrs.x = std::stod(mgrs_code.substr(GZD_ID_size, static_cast<int>(precision))) *
                 std::pow(
                         10, static_cast<int>(RosPospacBridge::MGRSPrecision::_1_METER) -
                             static_cast<int>(precision));  // set unit as [m]
        mgrs.y = std::stod(mgrs_code.substr(
                GZD_ID_size + static_cast<int>(precision), static_cast<int>(precision))) *
                 std::pow(
                         10, static_cast<int>(RosPospacBridge::MGRSPrecision::_1_METER) -
                             static_cast<int>(precision));  // set unit as [m]
        mgrs.z = gnss_stat_utm.z;                                 // set unit as [m]
    } catch (const GeographicLib::GeographicErr & err) {
        RCLCPP_ERROR_STREAM(logger, "Failed to convert from UTM to MGRS" << err.what());
    }
    return mgrs;
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros_pospac_bridge::RosPospacBridge)