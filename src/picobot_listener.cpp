#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <bitset>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

typedef enum state
{
  STOP,
  FORWARD,
  RIGHT,
  BACKWARD,
  LEFT
} state;

typedef struct sensor
{
  bool left;
  bool middle;
  bool right;
} sensor;

class PicobotListener : public rclcpp::Node
{
public:
  PicobotListener()
      : Node("picobot_listener"), count_(0)
  {
    sensors_sub = this->create_subscription<std_msgs::msg::Int8>("picobot/sensors", rclcpp::SensorDataQoS(),
                                                                 std::bind(&PicobotListener::sensors_callback, this, _1));
    cmd_vel_sub = this->create_subscription<std_msgs::msg::Int8>("picobot/cmd_vel", rclcpp::SensorDataQoS(),
                                                                 std::bind(&PicobotListener::cmd_vel_callback, this, _1));
  }

private:
  void sensors_callback(const std_msgs::msg::Int8 &msg)
  {
    std::bitset<6> data(msg.data); // Pos 0-2: ground(right-left), 3-5: wall(right-left)
    ground.right = bool(data[0]);
    ground.middle = bool(data[1]);
    ground.left = bool(data[2]);

    wall.right = !bool(data[3]);
    wall.middle = !bool(data[4]);
    wall.left = !bool(data[5]);

    RCLCPP_INFO(this->get_logger(), "WALL: %d %d %d --- GROUND %d %d %d", wall.left, wall.middle, wall.right, ground.left, ground.middle, ground.right);
  }
  void cmd_vel_callback(const std_msgs::msg::Int8 &msg)
  {
    std::string command;
    switch (msg.data)
    {
    case STOP:
      command = "STOP";
      break;
    case FORWARD:
      command = "FORWARD";
      break;
    case RIGHT:
      command = "RIGHT";
      break;
    case BACKWARD:
      command = "BACKWARD";
      break;
    case LEFT:
      command = "LEFT";
      break;

    default:
      break;
    }
    RCLCPP_INFO(this->get_logger(), "COMMAND : %d --> %s", msg.data, command.c_str());
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr cmd_vel_sub;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sensors_sub;
  size_t count_;
  sensor wall, ground;
  state picobot_state;
  //std_msgs::msg::Int8 message;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PicobotListener>());
  rclcpp::shutdown();
  return 0;
}