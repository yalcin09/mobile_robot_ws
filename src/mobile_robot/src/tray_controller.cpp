#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>

class TrayController : public rclcpp::Node
{
public:
    TrayController() : Node("tray_controller")
    {
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("tray_joint_states", 10);
        tray_position_pub_ = this->create_publisher<std_msgs::msg::Float64>("/tray_position_cmd", 10);
        
        tray_position_ = 0.0;
        tray_velocity_ = 0.0;
        tray_speed_ = 0.05;
        
        min_position_ = 0.0;
        max_position_ = 0.25;
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TrayController::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Tray Controller started");
        RCLCPP_INFO(this->get_logger(), "Use R/F keys to control the tray:");
        RCLCPP_INFO(this->get_logger(), "  R - Raise tray");
        RCLCPP_INFO(this->get_logger(), "  F - Lower tray");
        RCLCPP_INFO(this->get_logger(), "  T - Increase speed");
        RCLCPP_INFO(this->get_logger(), "  G - Decrease speed");
        RCLCPP_INFO(this->get_logger(), "  Space - Stop tray");
        RCLCPP_INFO(this->get_logger(), "  C - Exit");
        
        // Publish initial state
        publishJointState();
        publishTrayPosition();
        
        setupTerminal();
        startKeyboardThread();
    }
    
    ~TrayController()
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
        keyboard_thread_ = std::thread(&TrayController::keyboardLoop, this);
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
        switch (tolower(key))
        {
            case 'r':
                tray_velocity_ = tray_speed_;
                RCLCPP_INFO(this->get_logger(), "Raising tray (velocity: %.3f)", tray_velocity_);
                break;
            case 'f':
                tray_velocity_ = -tray_speed_;
                RCLCPP_INFO(this->get_logger(), "Lowering tray (velocity: %.3f)", tray_velocity_);
                break;
            case 't':
                tray_speed_ += 0.01;
                if (tray_speed_ > 0.1) tray_speed_ = 0.1;
                RCLCPP_INFO(this->get_logger(), "Speed increased to %.3f", tray_speed_);
                break;
            case 'g':
                tray_speed_ -= 0.01;
                if (tray_speed_ < 0.01) tray_speed_ = 0.01;
                RCLCPP_INFO(this->get_logger(), "Speed decreased to %.3f", tray_speed_);
                break;
            case ' ':
                tray_velocity_ = 0.0;
                RCLCPP_INFO(this->get_logger(), "Tray STOPPED");
                break;
            case 'c':
                tray_velocity_ = 0.0;
                publishJointState();
                RCLCPP_INFO(this->get_logger(), "Exiting tray controller");
                rclcpp::shutdown();
                break;
            default:
                break;
        }
    }
    
    void controlLoop()
    {
        auto current_time = this->now();
        double dt = 0.05;
        
        double new_position = tray_position_ + tray_velocity_ * dt;
        
        if (new_position < min_position_)
        {
            new_position = min_position_;
            tray_velocity_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Tray reached minimum position (%.3f)", min_position_);
        }
        else if (new_position > max_position_)
        {
            new_position = max_position_;
            tray_velocity_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Tray reached maximum position (%.3f)", max_position_);
        }
        
        tray_position_ = new_position;
        publishJointState();
        publishTrayPosition();
    }
    
    void publishJointState()
    {
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = this->now();
        joint_state.header.frame_id = "";
        
        joint_state.name = {"trayJoint"};
        joint_state.position = {tray_position_};
        joint_state.velocity = {tray_velocity_};
        joint_state.effort = {};
        
        joint_state_pub_->publish(joint_state);
        
        static auto last_log_time = this->now();
        auto current_time = this->now();
        
        if ((current_time - last_log_time).seconds() > 1.0)
        {
            RCLCPP_INFO(this->get_logger(), "Tray position: %.3f m (%.1f%%)", 
                       tray_position_, (tray_position_ / max_position_) * 100.0);
            last_log_time = current_time;
        }
    }
    
    void publishTrayPosition()
    {
        std_msgs::msg::Float64 position_msg;
        position_msg.data = tray_position_;
        tray_position_pub_->publish(position_msg);
        
        // Debug: Print published position occasionally
        static int debug_counter = 0;
        if (++debug_counter % 20 == 0) {  // Every 1 second (50ms * 20)
            RCLCPP_INFO(this->get_logger(), "Publishing tray position: %.3f to /tray_position_cmd", 
                       tray_position_);
        }
    }
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tray_position_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double tray_position_;
    double tray_velocity_;
    double tray_speed_;
    double min_position_;
    double max_position_;
    
    struct termios old_settings_;
    struct termios new_settings_;
    std::thread keyboard_thread_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrayController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}