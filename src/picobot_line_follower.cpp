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

typedef enum
{
  STOP,
  FORWARD,
  RIGHT,
  BACKWARD,
  LEFT,
  DUCK_LEFT,
  DUCK_RIGHT
} action;

typedef enum
{
  PARK,
  FOLLOW_LINE,
  TURN_RIGHT,
  TURN_LEFT,
  REACH_CENTER
} state;

typedef struct
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

    wall.right = !bool(data[3]);
    wall.middle = !bool(data[4]);
    wall.left = !bool(data[5]);

    // RCLCPP_INFO(this->get_logger(), "GROUND: %d %d %d", ground.left, ground.middle, ground.right);
  }

  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "STATE: %s --- WALL: %d %d %d --- GROUND: %d %d %d", picobot_state_string[picobot_state].c_str(), wall.left, wall.middle, wall.right, ground.left, ground.middle, ground.right);
    if (true && running) // case courante != arrivée
    {
      switch (picobot_state)
      {
      case PARK:
        if (!wall.right)
        {
          picobot_state = TURN_RIGHT;
        }
        else if (!wall.middle)
        {
          picobot_state = FOLLOW_LINE;
        }
        else if (!wall.left)
        {
          picobot_state = TURN_LEFT;
        }
        else
        {
          picobot_state = TURN_RIGHT;
        }

        break;
      case FOLLOW_LINE:
        if (ground.left && ground.middle && ground.right) // 1 1 1
        {
          picobot_action = STOP;
          picobot_state = PARK;
          message.data = picobot_action;
          publisher_->publish(message);
        }
        else if (!ground.left && ground.middle && !ground.right) // 0 1 0
        {
          picobot_action = FORWARD;
          message.data = picobot_action;
          publisher_->publish(message);
        }
        else if ((!ground.left && !ground.middle && ground.right)    // 0 0 1
                 || (!ground.left && ground.middle && ground.right)) // 0 1 1
        {
          picobot_action = RIGHT;
          message.data = picobot_action;
          publisher_->publish(message);
        }
        else if ((ground.left && !ground.middle && !ground.right)    // 1 0 0
                 || (ground.left && ground.middle && !ground.right)) // 1 1 0
        {
          picobot_action = LEFT;
          message.data = picobot_action;
          publisher_->publish(message);
        }
        break;
      case TURN_LEFT:
        if ((ground.left && !ground.middle && !ground.right)    // 1 0 0
            || (ground.left && ground.middle && !ground.right)) // 1 1 0
        {
          picobot_state = REACH_CENTER;
        }
        else
        {
          picobot_action = DUCK_LEFT;
          message.data = picobot_action;
          publisher_->publish(message);
        }
        break;
      case TURN_RIGHT:
        if ((!ground.left && !ground.middle && ground.right)    // 0 0 1
            || (!ground.left && ground.middle && ground.right)) // 0 1 1
        {
          picobot_state = REACH_CENTER;
        }
        else
        {
          picobot_action = DUCK_RIGHT;
          message.data = picobot_action;
          publisher_->publish(message);
        }
        break;

      case REACH_CENTER:
        if ((!ground.left && ground.middle && !ground.right)) // 0 1 0
        {
          picobot_action = STOP;
          picobot_state = FOLLOW_LINE;
          message.data = picobot_action;
          publisher_->publish(message);
        }
        else
        {
          publisher_->publish(message);
        }
        break;

      default:
        break;
      }
    }
    else
    {
      picobot_action = STOP;
      message.data = picobot_action;
      publisher_->publish(message);
      rclcpp::shutdown();
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sensors_sub;
  size_t count_;
  sensor wall, ground;
  action picobot_action = STOP;
  state picobot_state = PARK;
  std_msgs::msg::Int8 message;
  std::string picobot_state_string[5] = {"PARK", "FOLLOW_LINE", "TURN_RIGHT", "TURN_LEFT", "REACH_CENTER"};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  signal(SIGINT, quit);
  rclcpp::spin(std::make_shared<PicobotLineFollower>());
  rclcpp::shutdown();
  return 0;
}