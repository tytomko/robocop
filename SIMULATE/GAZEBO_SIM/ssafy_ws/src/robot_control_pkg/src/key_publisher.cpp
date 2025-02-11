/*
수정사항
- 토핑명 변경 "/robot_" + std::to_string(robot_num_) + "/key_input";
- robot_control_pkg패키지로 통합
*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <string>

class KeyPublisher : public rclcpp::Node
{
public:
    KeyPublisher() : Node("key_publisher")
    {
        // 1) 파라미터 선언
        this->declare_parameter<std::string>("robot_name", "default_robot");
        this->declare_parameter<int>("robot_number", -1);
        robot_name_ = this->get_parameter("robot_name").as_string();
        robot_num_ = this->get_parameter("robot_number").as_int();
        // 2) 발행 토픽 설정
        std::string topic_name = "/robot_" + std::to_string(robot_num_) + "/key_input";
        publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);

        RCLCPP_INFO(this->get_logger(), "Key Publisher Node Started. Publishing to topic: %s", topic_name.c_str());

        setupTerminal();
    }

    ~KeyPublisher()
    {
        restoreTerminal();
    }

    void spin()
    {
        char key;
        while (rclcpp::ok())
        {
            key = getKey();
            if (key != '\0')
            {
                auto message = std_msgs::msg::String();
                message.data = processKey(key);
                publisher_->publish(message);
                RCLCPP_INFO(this->get_logger(), "Published: '%s'", message.data.c_str());
            }
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::string robot_name_;
    int robot_num_;
    struct termios original_terminal_;

    void setupTerminal()
    {
        struct termios new_terminal;
        tcgetattr(STDIN_FILENO, &original_terminal_);
        new_terminal = original_terminal_;
        new_terminal.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal);
    }

    void restoreTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_terminal_);
    }

    char getKey()
    {
        char c;
        if (read(STDIN_FILENO, &c, 1) < 0)
        {
            return '\0';
        }
        return c;
    }

    std::string processKey(char key)
    {
        switch (key)
        {
        case ' ':
            return "SPACE";
        case 'I':
        case 'i':
            return "UP_LEFT";
        case 'O':
        case 'o':
            return "UP_RIGHT";
        case 'K':
        case 'k':
            return "DOWN_LEFT";
        case 'L':
        case 'l':
            return "DOWN_RIGHT";
        case '\033': // Arrow keys start with an escape sequence
            if (getKey() == '[')
            {
                switch (getKey())
                {
                case 'A':
                    return "UP";
                case 'B':
                    return "DOWN";
                case 'C':
                    return "RIGHT";
                case 'D':
                    return "LEFT";
                }
            }
            break;
        default:
            return std::string(1, key);
        }
        return "UNKNOWN";
    }
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyPublisher>();
    node->spin();
    rclcpp::shutdown();
    return 0;
}
