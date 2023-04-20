#include <functional>
#include <stdexcept>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>

#include <signal.h>
#include <stdio.h>

#include <termios.h>
#include <unistd.h>

static constexpr char KEYCODE_RIGHT = 0x43;
static constexpr char KEYCODE_LEFT = 0x44;
static constexpr char KEYCODE_UP = 0x41;
static constexpr char KEYCODE_DOWN = 0x42;
static constexpr char KEYCODE_SPACE = 0x20;
static constexpr char KEYCODE_Q = 0x71;
static constexpr char KEYCODE_W = 0x77;
static constexpr char KEYCODE_X = 0x78;

bool running = true;

class KeyboardReader final
{
public:
    KeyboardReader()
    {
        // get the console in raw mode
        if (tcgetattr(0, &cooked_) < 0)
        {
            throw std::runtime_error("Failed to get old console mode");
        }
        struct termios raw;
        memcpy(&raw, &cooked_, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);
        // Setting a new line, then end of file
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        raw.c_cc[VTIME] = 1;
        raw.c_cc[VMIN] = 0;
        if (tcsetattr(0, TCSANOW, &raw) < 0)
        {
            throw std::runtime_error("Failed to set new console mode");
        }
    }

    char readOne()
    {
        char c = 0;

        int rc = read(0, &c, 1);
        if (rc < 0)
        {
            throw std::runtime_error("read failed");
        }

        return c;
    }

    ~KeyboardReader()
    {
        tcsetattr(0, TCSANOW, &cooked_);
    }

private:
    struct termios cooked_;
};

class TeleopTurtle final
{
public:
    TeleopTurtle()
    {
        nh_ = rclcpp::Node::make_shared("picobot_keyboard");

        int_pub_ = nh_->create_publisher<std_msgs::msg::Int8>("picobot/cmd_vel", rclcpp::SensorDataQoS());
    }

    int keyLoop()
    {
        char c;

        std::thread{std::bind(&TeleopTurtle::spin, this)}.detach();

        puts("Reading from keyboard");
        puts("---------------------------");
        puts("Use arrow keys to move the picobot.");
        puts("Press W/X keys for duck left/right movement.");
        puts("'Q' to quit.");

        while (running)
        {
            // get the next event from the keyboard
            try
            {
                c = input_.readOne();
            }
            catch (const std::runtime_error &)
            {
                perror("read():");
                return -1;
            }
            int movement = -1;

            RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X\n", c);

            switch (c)
            {
            case KEYCODE_SPACE:
                RCLCPP_DEBUG(nh_->get_logger(), "STOP");
                movement = 0;
                break;
            case KEYCODE_UP:
                RCLCPP_DEBUG(nh_->get_logger(), "FORWARD");
                movement = 1;
                break;
            case KEYCODE_RIGHT:
                RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
                movement = 2;
                break;
            case KEYCODE_DOWN:
                RCLCPP_DEBUG(nh_->get_logger(), "BACKWARD");
                movement = 3;
                break;
            case KEYCODE_LEFT:
                RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
                movement = 4;
                break;
            case KEYCODE_W:
                RCLCPP_DEBUG(nh_->get_logger(), "DUCK LEFT");
                movement = 5;
                break;
            case KEYCODE_X:
                RCLCPP_DEBUG(nh_->get_logger(), "DUCK RIGHT");
                movement = 6;
                break;
            case KEYCODE_Q:
                RCLCPP_DEBUG(nh_->get_logger(), "QUIT");
                running = false;
                movement = 0;
                break;
            default:
                RCLCPP_DEBUG(nh_->get_logger(), std::to_string((int)c).c_str());
                // This can happen if the read returned when there was no data, or
                // another key was pressed.  In these cases, just silently ignore the
                // press.
                break;
            }

            if (movement != -1)
            {
                std_msgs::msg::Int8 msg_int;
                msg_int.data = movement;
                int_pub_->publish(msg_int);
            }
        }

        return 0;
    }

private:
    void spin()
    {
        rclcpp::spin(nh_);
    }

    rclcpp::Node::SharedPtr nh_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr int_pub_;

    KeyboardReader input_;
};

void quit(int sig)
{
    (void)sig;
    running = false;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, quit);

    TeleopTurtle teleop_turtle;

    int rc = teleop_turtle.keyLoop();

    rclcpp::shutdown();

    return rc;
}