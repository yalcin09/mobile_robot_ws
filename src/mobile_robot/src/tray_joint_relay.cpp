#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class TrayJointRelay : public rclcpp::Node
{
public:
    TrayJointRelay() : Node("tray_joint_relay")
    {
        // Subscribe to tray joint states and gazebo joint states
        tray_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "tray_joint_states", 10,
            std::bind(&TrayJointRelay::trayCallback, this, std::placeholders::_1));
        
        gazebo_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&TrayJointRelay::gazeboCallback, this, std::placeholders::_1));
        
        // Publish merged joint states
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states_merged", 10);
        
        // Timer to publish at regular intervals
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TrayJointRelay::publishMerged, this));
        
        RCLCPP_INFO(this->get_logger(), "Tray Joint Relay started");
    }

private:
    void trayCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        tray_joint_state_ = *msg;
        tray_received_ = true;
    }
    
    void gazeboCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        gazebo_joint_state_ = *msg;
        gazebo_received_ = true;
    }
    
    void publishMerged()
    {
        if (!gazebo_received_) return;
        
        sensor_msgs::msg::JointState merged_msg = gazebo_joint_state_;
        merged_msg.header.stamp = this->now();
        
        // Add tray joint if received
        if (tray_received_)
        {
            for (size_t i = 0; i < tray_joint_state_.name.size(); ++i)
            {
                // Check if this joint already exists in gazebo joints
                bool exists = false;
                for (size_t j = 0; j < merged_msg.name.size(); ++j)
                {
                    if (merged_msg.name[j] == tray_joint_state_.name[i])
                    {
                        // Update existing joint
                        if (i < tray_joint_state_.position.size() && j < merged_msg.position.size())
                            merged_msg.position[j] = tray_joint_state_.position[i];
                        if (i < tray_joint_state_.velocity.size() && j < merged_msg.velocity.size())
                            merged_msg.velocity[j] = tray_joint_state_.velocity[i];
                        exists = true;
                        break;
                    }
                }
                
                // Add new joint if it doesn't exist
                if (!exists)
                {
                    merged_msg.name.push_back(tray_joint_state_.name[i]);
                    if (i < tray_joint_state_.position.size())
                        merged_msg.position.push_back(tray_joint_state_.position[i]);
                    if (i < tray_joint_state_.velocity.size())
                        merged_msg.velocity.push_back(tray_joint_state_.velocity[i]);
                    if (i < tray_joint_state_.effort.size())
                        merged_msg.effort.push_back(tray_joint_state_.effort[i]);
                }
            }
        }
        
        joint_pub_->publish(merged_msg);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr tray_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gazebo_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    sensor_msgs::msg::JointState tray_joint_state_;
    sensor_msgs::msg::JointState gazebo_joint_state_;
    bool tray_received_ = false;
    bool gazebo_received_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrayJointRelay>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}