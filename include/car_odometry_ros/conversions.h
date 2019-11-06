#pragma once

#include <car_odometry/odometry_state.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace car_odometry {

// toMsg
//
// @param state: Odometry state to convert
// @param msg: Quaternion message populated from the odometry state orientation
void toMsg(const OdometryState& state, geometry_msgs::Quaternion* msg);

// toMsg
//
// @param state: Odometry state to convert
// @param msg: Pose message populated from the odometry state pose
void toMsg(const OdometryState& state, geometry_msgs::Pose* msg);

// toMsg
//
// @param state: Odometry state to convert
// @param msg: Twist message populated from the odometry state velocity
void toMsg(const OdometryState& state, geometry_msgs::Twist* msg);

// toMsg
//
// @param state: Odometry state to convert
// @param msg: Odometry message populated from the odometry state
void toMsg(const OdometryState& state, nav_msgs::Odometry* msg);

// toMsg
//
// @param state: Odometry state to convert
// @param msg: Transform message populated from the odometry state
void toMsg(const OdometryState& state, geometry_msgs::TransformStamped* msg);

// fromMsg
//
// @param msg: Quaternion message to convert
// @return: State with orientation data populated from msg
void fromMsg(const geometry_msgs::Quaternion& msg, OdometryState* state);

// fromMsg
//
// @param msg: Pose message to convert
// @return: State with pose data populated from msg
void fromMsg(const geometry_msgs::Pose& msg, OdometryState* state);

// fromMsg
//
// @param msg: Twist message to convert
// @return: State with velocity data populated from msg
void fromMsg(const geometry_msgs::Twist& msg, OdometryState* state);

// fromMsg
//
// @param msg: Odometry message to convert
// @return: State populated from msg
void fromMsg(const nav_msgs::Odometry& msg, OdometryState* state);

// fromMsg
//
// @param msg: Transform message to convert
// @return: State populated from msg
void fromMsg(const geometry_msgs::TransformStamped& msg, OdometryState* state);

// parseCovarianceParameter
//
// Parses a parameter list of covarience values as strings and converts them to a floating point
// format will handle scientific notation and lack of decimals.
//
// @param val_strings: Strings to parse
// @return: Floating point representation of parameter values.
std::vector<double> parseCovarianceParameter(const std::vector<std::string>& val_strings);

}  // namespace car_odometry
