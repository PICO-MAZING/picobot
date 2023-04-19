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

void quit(int signal)
{
  (void)signal;
  running = false;
}

class PicobotController : public rclcpp::Node
{
public:
  PicobotController()
      : Node("picobot_controller"), count_(0)
  {
    sensors_sub = this->create_subscription<std_msgs::msg::Int8>("picobot/sensors", rclcpp::SensorDataQoS(),
                                                                 std::bind(&PicobotController::sensors_callback, this, _1));
    publisher_ = this->create_publisher<std_msgs::msg::Int8>("picobot/cmd_vel", rclcpp::SensorDataQoS());
    timer_ = this->create_wall_timer(
        500ms, std::bind(&PicobotController::timer_callback, this));
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

  void timer_callback()
  {
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
    if (true && running) // case courante != arrivée
    {
      if (ground.left && ground.middle && ground.right)
      {
        picobot_state = STOP;
        message.data = picobot_state;
        publisher_->publish(message);
        // sleep(2);
      }
      else
      {
        if (!wall.right)
        {
          picobot_state = RIGHT;
          message.data = picobot_state;
          publisher_->publish(message);
        }
        else if (!wall.middle)
        {
          picobot_state = FORWARD;
          message.data = picobot_state;
          publisher_->publish(message);
          // sleep(2);
        }
        else if (!wall.left)
        {
          picobot_state = LEFT;
          message.data = picobot_state;
          publisher_->publish(message);
          // sleep(2);
        }
        else
        {
          picobot_state = FORWARD;
          message.data = picobot_state;
          publisher_->publish(message);
        }
      }
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
  rclcpp::spin(std::make_shared<PicobotController>());
  rclcpp::shutdown();
  return 0;
}