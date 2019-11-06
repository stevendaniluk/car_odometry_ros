#pragma once

#include <car_odometry/car_odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

namespace car_odometry {

// Parameters for ROS node settings
struct CarOdometryRosParameters : public CarOdometryParameters {
    // Topic to publish odometry messages on
    std::string odom_topic = "odometry";
    // Vehicle frame the odometry estimate is computed for
    std::string base_frame = "base_link";
    // World frame the odometry pose is expressed in
    std::string world_frame = "odom";
    // Flag for if the transform should be published
    bool broadcast_tf = true;
    // Covariance to assign to the diagonal entries in the pose covariance matrix
    std::vector<double> pose_covariance_diagonal = std::vector<double>(6, 1e-3);
    // Covariance to assign to the diagonal entries in the twist covariance matrix
    std::vector<double> twist_covariance_diagonal = std::vector<double>(6, 1e-3);
};

// CarOdometryRos
//
// ROS wrapper around the CarOdometry class that will publish odometry and transform messages with
// state updates.
//
class CarOdometryRos : public CarOdometry {
  public:
    // Constructor
    //
    // @param nh: Nodehandle to publish with
    // @node_params: Parameters for odometry computation
    CarOdometryRos(const ros::NodeHandle& nh, const CarOdometryRosParameters& params);

    // Version of updateState that accepts ROS time
    void updateState(const EncoderPulses& pulses, double delta_FL, double delta_FR,
                     const ros::Time& time);

    // broadcast
    //
    // Publishes the current odometry state and transform (if enabled).
    void broadcast();

  protected:
    // Nodehandle to publish with
    ros::NodeHandle nh_;
    // Pubslisher of odometry message
    ros::Publisher pub_odom_;
    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // Odometry message to update and publish (has pre set data in it)
    nav_msgs::Odometry odom_msg_;
    // Transform to broadcast (has pre set data in it)
    geometry_msgs::TransformStamped tf_msg_;
    // Flag for if the transform should be broadcastes
    bool broadcast_tf_;
};

}  // namespace car_odometry
