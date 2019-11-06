#include <car_odometry_ros/car_odometry_ros.h>
#include <car_odometry_ros/conversions.h>
#include <boost/assign.hpp>

namespace car_odometry {

CarOdometryRos::CarOdometryRos(const ros::NodeHandle& nh, const CarOdometryRosParameters& params)
    : CarOdometry::CarOdometry(static_cast<CarOdometryParameters>(params))
    , nh_(nh)
    , broadcast_tf_(params.broadcast_tf) {
    // Initialize our message frame data
    odom_msg_.header.frame_id = params.world_frame;
    odom_msg_.child_frame_id = params.base_frame;

    tf_msg_.header.frame_id = params.world_frame;
    tf_msg_.child_frame_id = params.base_frame;

    // Initialize the odometry covariance matrices
    ROS_ASSERT(params.pose_covariance_diagonal.size() == 6);
    ROS_ASSERT(params.twist_covariance_diagonal.size() == 6);
    std::vector<double> cov;

    // clang-format off
    cov = params.pose_covariance_diagonal;
    odom_msg_.pose.covariance = boost::assign::list_of
        (cov[0]) (0)  (0)  (0)  (0)  (0)
        (0)  (cov[1]) (0)  (0)  (0)  (0)
        (0)  (0)  (cov[2]) (0)  (0)  (0)
        (0)  (0)  (0)  (cov[3]) (0)  (0)
        (0)  (0)  (0)  (0)  (cov[4]) (0)
        (0)  (0)  (0)  (0)  (0)  (cov[5]);

    cov = params.twist_covariance_diagonal;
    odom_msg_.twist.covariance = boost::assign::list_of
        (cov[0]) (0)  (0)  (0)  (0)  (0)
        (0)  (cov[1]) (0)  (0)  (0)  (0)
        (0)  (0)  (cov[2]) (0)  (0)  (0)
        (0)  (0)  (0)  (cov[3]) (0)  (0)
        (0)  (0)  (0)  (0)  (cov[4]) (0)
        (0)  (0)  (0)  (0)  (0)  (cov[5]);
    // clang-format on

    // Setup the publisher
    pub_odom_ = nh_.advertise<nav_msgs::Odometry>(params.odom_topic, 50);

    // Setup the transform broadcaster
    if (broadcast_tf_) {
        tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster());
    }

    ROS_INFO("Car odometry initialized");
}

void CarOdometryRos::updateState(const EncoderPulses& pulses, double delta_FL, double delta_FR,
                                 const ros::Time& time) {
    CarOdometry::updateState(pulses, delta_FL, delta_FR, time.toSec());
}

void CarOdometryRos::broadcast() {
    toMsg(getState(), &odom_msg_);
    pub_odom_.publish(odom_msg_);

    if (broadcast_tf_) {
        toMsg(getState(), &tf_msg_);
        tf_broadcaster_->sendTransform(tf_msg_);
    }
}

}  // namespace car_odometry
