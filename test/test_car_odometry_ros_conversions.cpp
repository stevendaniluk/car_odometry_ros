#include <car_odometry_ros/conversions.h>
#include <gtest/gtest.h>

using namespace car_odometry;

// Test fixture which contains a sample state to convert
class CarOdometryRosConversions : public ::testing::Test {
  protected:
    void SetUp() override {
        // Provide some dummy values for each data vector
        state.timestamp = 1.23456789;
        state.position.x() = 3.14159;
        state.position.y() = 2.71828;
        state.psi = 0.123;
        state.psi_dot = 0.456;
        state.velocity = 0.789;
    }

    OdometryState state;
};

TEST_F(CarOdometryRosConversions, Quaternion) {
    OdometryState state_converted;
    geometry_msgs::Quaternion msg;
    toMsg(state, &msg);
    fromMsg(msg, &state_converted);

    EXPECT_FLOAT_EQ(state.psi, state_converted.psi);
}

TEST_F(CarOdometryRosConversions, Pose) {
    OdometryState state_converted;
    geometry_msgs::Pose msg;
    toMsg(state, &msg);
    fromMsg(msg, &state_converted);

    EXPECT_FLOAT_EQ(state.position.x(), state_converted.position.x());
    EXPECT_FLOAT_EQ(state.position.y(), state_converted.position.y());
    EXPECT_FLOAT_EQ(state.psi, state_converted.psi);
}

TEST_F(CarOdometryRosConversions, Twist) {
    OdometryState state_converted;
    geometry_msgs::Twist msg;
    toMsg(state, &msg);
    fromMsg(msg, &state_converted);

    EXPECT_FLOAT_EQ(state.velocity, state_converted.velocity);
    EXPECT_FLOAT_EQ(state.psi_dot, state_converted.psi_dot);
}

TEST_F(CarOdometryRosConversions, Odometry) {
    OdometryState state_converted;
    nav_msgs::Odometry msg;
    toMsg(state, &msg);
    fromMsg(msg, &state_converted);

    EXPECT_FLOAT_EQ(state.timestamp, state_converted.timestamp);
    EXPECT_FLOAT_EQ(state.position.x(), state_converted.position.x());
    EXPECT_FLOAT_EQ(state.position.y(), state_converted.position.y());
    EXPECT_FLOAT_EQ(state.psi, state_converted.psi);
    EXPECT_FLOAT_EQ(state.velocity, state_converted.velocity);
    EXPECT_FLOAT_EQ(state.psi_dot, state_converted.psi_dot);
}

TEST_F(CarOdometryRosConversions, TransformStamped) {
    OdometryState state_converted;
    geometry_msgs::TransformStamped msg;
    toMsg(state, &msg);
    fromMsg(msg, &state_converted);

    EXPECT_FLOAT_EQ(state.timestamp, state_converted.timestamp);
    EXPECT_FLOAT_EQ(state.position.x(), state_converted.position.x());
    EXPECT_FLOAT_EQ(state.position.y(), state_converted.position.y());
    EXPECT_FLOAT_EQ(state.psi, state_converted.psi);
}
