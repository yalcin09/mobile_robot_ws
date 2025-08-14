#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>

class TeleopKeyboard : public rclcpp::Node
{
public:
    TeleopKeyboard() : Node("teleop_keyboard")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        linear_speed_ = 0.5;
        angular_speed_ = 1.0;
        
        linear_vel_ = 0.0;
        angular_vel_ = 0.0;
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TeleopKeyboard::publishVelocity, this));
        
        RCLCPP_INFO(this->get_logger(), "Teleop Keyboard Node started");
        RCLCPP_INFO(this->get_logger(), "Use WASD keys to control the robot:");
        RCLCPP_INFO(this->get_logger(), "  W - Forward");
        RCLCPP_INFO(this->get_logger(), "  S - Backward");
        RCLCPP_INFO(this->get_logger(), "  A - Turn Left");
        RCLCPP_INFO(this->get_logger(), "  D - Turn Right");
        RCLCPP_INFO(this->get_logger(), "  Q - Increase Speed");
        RCLCPP_INFO(this->get_logger(), "  E - Decrease Speed");
        RCLCPP_INFO(this->get_logger(), "  Space - Stop");
        RCLCPP_INFO(this->get_logger(), "  X - Exit");
        
        setupTerminal();
        startKeyboardThread();
    }
    
    ~TeleopKeyboard()
    {
        restoreTerminal();
    }

private:
    void setupTerminal()
    {
        tcgetattr(STDIN_FILENO, &old_settings_);
        new_settings_ = old_settings_;
        new_settings_.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings_);
    }
    
    void restoreTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_settings_);
    }
    
    void startKeyboardThread()
    {
        keyboard_thread_ = std::thread(&TeleopKeyboard::keyboardLoop, this);
    }
    
    void keyboardLoop()
    {
        char key;
        while (rclcpp::ok())
        {
            if (read(STDIN_FILENO, &key, 1) == 1)
            {
                processKey(key);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    void processKey(char key)
    {
        bool velocity_changed = false;
        
        switch (tolower(key))
        {
            case 'w':
                linear_vel_ = linear_speed_;
                angular_vel_ = 0.0;
                velocity_changed = true;
                RCLCPP_INFO(this->get_logger(), "Moving Forward (Linear: %.2f)", linear_vel_);
                break;
            case 's':
                linear_vel_ = -linear_speed_;
                angular_vel_ = 0.0;
                velocity_changed = true;
                RCLCPP_INFO(this->get_logger(), "Moving Backward (Linear: %.2f)", linear_vel_);
                break;
            case 'a':
                linear_vel_ = 0.0;
                angular_vel_ = angular_speed_;
                velocity_changed = true;
                RCLCPP_INFO(this->get_logger(), "Turning Left (Angular: %.2f)", angular_vel_);
                break;
            case 'd':
                linear_vel_ = 0.0;
                angular_vel_ = -angular_speed_;
                velocity_changed = true;
                RCLCPP_INFO(this->get_logger(), "Turning Right (Angular: %.2f)", angular_vel_);
                break;
            case 'q':
                linear_speed_ += 0.1;
                angular_speed_ += 0.1;
                if (linear_speed_ > 1.0) linear_speed_ = 1.0;
                if (angular_speed_ > 2.0) angular_speed_ = 2.0;
                RCLCPP_INFO(this->get_logger(), "Speed increased - Linear: %.2f, Angular: %.2f", 
                           linear_speed_, angular_speed_);
                break;
            case 'e':
                linear_speed_ -= 0.1;
                angular_speed_ -= 0.1;
                if (linear_speed_ < 0.1) linear_speed_ = 0.1;
                if (angular_speed_ < 0.1) angular_speed_ = 0.1;
                RCLCPP_INFO(this->get_logger(), "Speed decreased - Linear: %.2f, Angular: %.2f", 
                           linear_speed_, angular_speed_);
                break;
            case ' ':
                linear_vel_ = 0.0;
                angular_vel_ = 0.0;
                velocity_changed = true;
                RCLCPP_INFO(this->get_logger(), "STOP");
                break;
            case 'x':
                linear_vel_ = 0.0;
                angular_vel_ = 0.0;
                publishVelocity();
                RCLCPP_INFO(this->get_logger(), "Exiting teleop keyboard");
                rclcpp::shutdown();
                break;
            default:
                break;
        }
        
        if (velocity_changed)
        {
            publishVelocity();
        }
    }
    
    void publishVelocity()
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = linear_vel_;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = angular_vel_;
        
        cmd_vel_pub_->publish(twist);
        
        // Debug: Print published velocity
        if (linear_vel_ != 0.0 || angular_vel_ != 0.0) {
            RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear=%.2f, angular=%.2f", 
                       linear_vel_, angular_vel_);
        }
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double linear_speed_;
    double angular_speed_;
    double linear_vel_;
    double angular_vel_;
    
    struct termios old_settings_;
    struct termios new_settings_;
    std::thread keyboard_thread_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopKeyboard>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}