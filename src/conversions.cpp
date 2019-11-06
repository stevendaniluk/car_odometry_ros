#include <car_odometry_ros/conversions.h>
#include <ros/assert.h>
#include <Eigen/Geometry>

namespace car_odometry {

void toMsg(const OdometryState& state, geometry_msgs::Quaternion* msg) {
    Eigen::Quaterniond quat =
        (Eigen::Quaterniond)Eigen::AngleAxisd(state.psi, Eigen::Vector3d::UnitZ());
    msg->x = quat.x();
    msg->y = quat.y();
    msg->z = quat.z();
    msg->w = quat.w();
}

void toMsg(const OdometryState& state, geometry_msgs::Pose* msg) {
    msg->position.x = state.position.x();
    msg->position.y = state.position.y();
    msg->position.z = 0.0;

    toMsg(state, &(msg->orientation));
}

void toMsg(const OdometryState& state, geometry_msgs::Twist* msg) {
    msg->linear.x = state.velocity;
    msg->linear.y = 0;
    msg->linear.z = 0;

    msg->angular.x = 0;
    msg->angular.y = 0;
    msg->angular.z = state.psi_dot;
}

void toMsg(const OdometryState& state, nav_msgs::Odometry* msg) {
    msg->header.stamp = ros::Time(state.timestamp);
    toMsg(state, &(msg->pose.pose));
    toMsg(state, &(msg->twist.twist));
}

void toMsg(const OdometryState& state, geometry_msgs::TransformStamped* msg) {
    msg->header.stamp = ros::Time(state.timestamp);
    msg->transform.translation.x = state.position.x();
    msg->transform.translation.y = state.position.y();
    msg->transform.translation.z = 0.0;

    toMsg(state, &(msg->transform.rotation));
}

void fromMsg(const geometry_msgs::Quaternion& msg, OdometryState* state) {
    const Eigen::Quaterniond quat(msg.w, msg.x, msg.y, msg.z);
    state->psi = Eigen::Rotation2Dd(quat.toRotationMatrix().topLeftCorner<2, 2>()).angle();
}

void fromMsg(const geometry_msgs::Pose& msg, OdometryState* state) {
    state->position.x() = msg.position.x;
    state->position.y() = msg.position.y;

    fromMsg(msg.orientation, state);
}

void fromMsg(const geometry_msgs::Twist& msg, OdometryState* state) {
    state->velocity = msg.linear.x;
    state->psi_dot = msg.angular.z;
}

void fromMsg(const nav_msgs::Odometry& msg, OdometryState* state) {
    state->timestamp = msg.header.stamp.toSec();
    fromMsg(msg.pose.pose, state);
    fromMsg(msg.twist.twist, state);
}

void fromMsg(const geometry_msgs::TransformStamped& msg, OdometryState* state) {
    state->timestamp = msg.header.stamp.toSec();
    state->position.x() = msg.transform.translation.x;
    state->position.y() = msg.transform.translation.y;
    fromMsg(msg.transform.rotation, state);
}

std::vector<double> parseCovarianceParameter(const std::vector<std::string>& val_strings) {
    std::vector<double> cov_vals(6);

    ROS_ASSERT(val_strings.size() == 6);
    for (int i = 0; i < 6; ++i) {
        std::istringstream istr(val_strings[i]);
        istr >> cov_vals[i];
    }

    return cov_vals;
}

}  // namespace car_odometry
