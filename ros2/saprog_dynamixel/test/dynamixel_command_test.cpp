#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "saprog_dynamixel/dynamixel_readback_node.hpp"

namespace saprog_dynamixel
{
struct DynamixelReadbackNodeTestPeer
{
  static void release(DynamixelReadbackNode &node)
  {
    auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
    node.release_all_torque(std::make_shared<std_srvs::srv::Trigger::Request>(), response);
    ASSERT_TRUE(response->success) << response->message;
  }

  static int64_t release_stamp(DynamixelReadbackNode &node)
  { return node.last_release_stamp_.nanoseconds(); }

  static int64_t release_system_stamp(DynamixelReadbackNode &node)
  { return node.last_release_system_stamp_; }

  static void position(DynamixelReadbackNode &node, int32_t sec, uint32_t nanosec)
  {
    auto message = std::make_shared<sensor_msgs::msg::JointState>();
    message->header.stamp.sec = sec;
    message->header.stamp.nanosec = nanosec;
    message->name = {"servo_11"};
    message->position = {0.2};
    node.command_positions(message);
  }

  static void position(DynamixelReadbackNode &node, int64_t stamp)
  { position(node, static_cast<int32_t>(stamp / 1000000000), stamp % 1000000000); }

  static bool torque(DynamixelReadbackNode &node, bool enabled, int64_t source_stamp)
  {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;
    auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
    node.set_servo_torque(11, source_stamp, request, response);
    return response->success;
  }
};

class DynamixelCommandTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides({
      rclcpp::Parameter("device_name", "fake"),
      rclcpp::Parameter("baud_rate", 1000000),
      rclcpp::Parameter("protocol_version", 1.0),
      rclcpp::Parameter("servo_ids", std::vector<int64_t>{11}),
      rclcpp::Parameter("zero_positions", std::vector<int64_t>{512}),
      rclcpp::Parameter("directions", std::vector<int64_t>{1}),
      rclcpp::Parameter("poll_rate", 20.0),
      rclcpp::Parameter("moving_speed", 100),
      rclcpp::Parameter("torque_limit", 500),
      rclcpp::Parameter("status_return_level", 1),
    });
    node = std::make_shared<DynamixelReadbackNode>(options);
    writes().clear();
  }

  static std::vector<std::pair<uint16_t, uint16_t>> &writes()
  { return dynamixel::PacketHandler::getPacketHandler(1.0)->writes; }
  void release() { DynamixelReadbackNodeTestPeer::release(*node); writes().clear(); }
  std::shared_ptr<DynamixelReadbackNode> node;
};

TEST_F(DynamixelCommandTest, AcceptsUnstampedPositionsBeforeRelease)
{
  DynamixelReadbackNodeTestPeer::position(*node, 0);
  EXPECT_EQ(writes().size(), 4u);
}

TEST_F(DynamixelCommandTest, ReleaseRejectsUnstampedOlderAndEqualPositions)
{
  release();
  auto stamp = DynamixelReadbackNodeTestPeer::release_stamp(*node);
  for (auto value : {int64_t{0}, stamp - 1, stamp}) {
    DynamixelReadbackNodeTestPeer::position(*node, value);
  }
  EXPECT_TRUE(writes().empty());
  DynamixelReadbackNodeTestPeer::position(*node, stamp + 1);
  EXPECT_EQ(writes().size(), 4u);
  EXPECT_EQ(writes().back(), std::make_pair(uint16_t{24}, uint16_t{1}));
}

TEST_F(DynamixelCommandTest, MalformedTimestampsNeverThrowOrWrite)
{
  for (bool released : {false, true}) {
    if (released) { release(); }
    EXPECT_NO_THROW(DynamixelReadbackNodeTestPeer::position(*node, -1, 0));
    EXPECT_NO_THROW(DynamixelReadbackNodeTestPeer::position(*node, 1, 1000000000u));
    EXPECT_TRUE(writes().empty());
  }
}

TEST_F(DynamixelCommandTest, ReleaseRejectsOldOrMissingTorqueEnableTimestamps)
{
  release();
  auto stamp = DynamixelReadbackNodeTestPeer::release_system_stamp(*node);
  for (auto value : {int64_t{0}, stamp - 1, stamp}) {
    EXPECT_FALSE(DynamixelReadbackNodeTestPeer::torque(*node, true, value));
  }
  EXPECT_TRUE(writes().empty());
  EXPECT_TRUE(DynamixelReadbackNodeTestPeer::torque(*node, true, stamp + 1));
  ASSERT_EQ(writes().size(), 1u);
  EXPECT_EQ(writes().back(), std::make_pair(uint16_t{24}, uint16_t{1}));
}

TEST_F(DynamixelCommandTest, TorqueDisableRemainsAvailableAfterRelease)
{
  release();
  EXPECT_TRUE(DynamixelReadbackNodeTestPeer::torque(*node, false, 0));
  ASSERT_EQ(writes().size(), 1u);
  EXPECT_EQ(writes().back(), std::make_pair(uint16_t{24}, uint16_t{0}));
}

TEST_F(DynamixelCommandTest, TorqueBarrierUsesSystemTimeWhenRosTimeIsPaused)
{
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("use_sim_time", true)).successful);
  // TimeSource applies runtime parameter changes through its parameter event
  // subscription; wait for that callback before checking the paused ROS clock.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (node->now().nanoseconds() != 0 && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(node->now().nanoseconds(), 0);
  const auto queued_stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds();
  release();
  EXPECT_FALSE(DynamixelReadbackNodeTestPeer::torque(*node, true, queued_stamp));
  EXPECT_TRUE(writes().empty());
  DynamixelReadbackNodeTestPeer::position(*node, 1);
  EXPECT_EQ(writes().size(), 4u);
}

TEST_F(DynamixelCommandTest, RepeatedReleaseAdvancesTheBarrier)
{
  release();
  const auto old_stamp = DynamixelReadbackNodeTestPeer::release_stamp(*node) + 1;
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  release();
  DynamixelReadbackNodeTestPeer::position(*node, old_stamp);
  EXPECT_TRUE(writes().empty());
}

TEST_F(DynamixelCommandTest, RealServiceRejectsQueuedEnableAndAcceptsANewRequest)
{
  using namespace std::chrono_literals;
  auto client_node = std::make_shared<rclcpp::Node>("torque_test_client");
  auto client = client_node->create_client<std_srvs::srv::SetBool>("/saprog/servo_11/torque");
  ASSERT_TRUE(client->wait_for_service(5s));
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  auto queued = client->async_send_request(request);
  std::this_thread::sleep_for(20ms);
  release();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(client_node);
  ASSERT_EQ(executor.spin_until_future_complete(queued, 5s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(queued.get()->success);
  EXPECT_TRUE(writes().empty());
  auto fresh = client->async_send_request(request);
  ASSERT_EQ(executor.spin_until_future_complete(fresh, 5s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(fresh.get()->success);
  ASSERT_EQ(writes().size(), 1u);
  EXPECT_EQ(writes().back(), std::make_pair(uint16_t{24}, uint16_t{1}));
}
}  // namespace saprog_dynamixel
