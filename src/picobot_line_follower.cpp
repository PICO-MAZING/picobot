#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <bitset>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
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

bool running = true;
bool tourner = false;

void quit(int signal)
{
  (void)signal;
  running = false;
}

class PicobotLineFollower : public rclcpp::Node
{
public:
  PicobotLineFollower()
      : Node("picobot_line_follower"), count_(0)
  {
    sensors_sub = this->create_subscription<std_msgs::msg::Int8>("picobot/sensors", rclcpp::SensorDataQoS(),
                                                                 std::bind(&PicobotLineFollower::sensors_callback, this, _1));
    publisher_ = this->create_publisher<std_msgs::msg::Int8>("picobot/cmd_vel", rclcpp::SensorDataQoS());
    timer_ = this->create_wall_timer(
        500ms, std::bind(&PicobotLineFollower::timer_callback, this));
  }

private:
  void sensors_callback(const std_msgs::msg::Int8 &msg)
  {
    std::bitset<6> data(msg.data); // Pos 0-2: ground(right-left), 3-5: wall(right-left)
    ground.right = bool(data[0]);
    ground.middle = bool(data[1]);
    ground.left = bool(data[2]);

    RCLCPP_INFO(this->get_logger(), "GROUND: %d %d %d", ground.left, ground.middle, ground.right);
  }

  void timer_callback()
  {
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
    if (true && running && !tourner) // case courante != arrivée
    {

      if (ground.left && ground.middle && ground.right)
      {
        picobot_state = STOP;
        tourner = true;
        message.data = picobot_state;
        publisher_->publish(message);
      }
      else if (!ground.left && ground.middle && !ground.right)
      {
        picobot_state = FORWARD;
        message.data = picobot_state;
        publisher_->publish(message);
      }
      else if (!ground.left && !ground.middle && ground.right)
      {
        picobot_state = RIGHT;
        message.data = picobot_state;
        publisher_->publish(message);
      }
      else if (ground.left && !ground.middle && !ground.right)
      {
        picobot_state = LEFT;
        message.data = picobot_state;
        publisher_->publish(message);
      }
    }
    else if (true && tourner)
    {
      tourner = false;
      picobot_state = RIGHT;
      message.data = picobot_state;
      publisher_->publish(message);
    }
    else
    {
      picobot_state = STOP;
      message.data = picobot_state;
      publisher_->publish(message);
      rclcpp::shutdown();
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sensors_sub;
  size_t count_;
  sensor wall, ground;
  state picobot_state = STOP;
  std_msgs::msg::Int8 message;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  signal(SIGINT, quit);
  rclcpp::spin(std::make_shared<PicobotLineFollower>());
  rclcpp::shutdown();
  return 0;
}