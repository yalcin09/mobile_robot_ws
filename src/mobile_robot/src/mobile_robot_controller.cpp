#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class MobileRobotController : public rclcpp::Node
{
public:
    MobileRobotController() : Node("mobile_robot_controller")
    {
        wheel_radius_ = 0.1;
        wheel_base_ = 0.65;
        
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        
        left_wheel_pos_ = 0.0;
        right_wheel_pos_ = 0.0;
        last_time_ = this->now();
        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, 
            std::bind(&MobileRobotController::cmdVelCallback, this, std::placeholders::_1));
        
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), 
            std::bind(&MobileRobotController::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Mobile Robot Controller started");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        linear_vel_ = msg->linear.x;
        angular_vel_ = msg->angular.z;
    }
    
    void controlLoop()
    {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        
        if (dt <= 0.0) return;
        
        double left_wheel_vel = (linear_vel_ - angular_vel_ * wheel_base_ / 2.0) / wheel_radius_;
        double right_wheel_vel = (linear_vel_ + angular_vel_ * wheel_base_ / 2.0) / wheel_radius_;
        
        left_wheel_pos_ += left_wheel_vel * dt;
        right_wheel_pos_ += right_wheel_vel * dt;
        
        double delta_left = left_wheel_vel * wheel_radius_ * dt;
        double delta_right = right_wheel_vel * wheel_radius_ * dt;
        double delta_s = (delta_left + delta_right) / 2.0;
        double delta_theta = (delta_right - delta_left) / wheel_base_;
        
        x_ += delta_s * cos(theta_ + delta_theta / 2.0);
        y_ += delta_s * sin(theta_ + delta_theta / 2.0);
        theta_ += delta_theta;
        
        theta_ = atan2(sin(theta_), cos(theta_));
        
        publishJointStates(current_time);
        publishOdometry(current_time);
        publishTransform(current_time);
        
        last_time_ = current_time;
    }
    
    void publishJointStates(const rclcpp::Time& time)
    {
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = time;
        joint_state.header.frame_id = "";
        
        joint_state.name = {"leftWheelJoint", "rightWheelJoint"};
        joint_state.position = {left_wheel_pos_, right_wheel_pos_};
        joint_state.velocity = {linear_vel_ / wheel_radius_, linear_vel_ / wheel_radius_};
        joint_state.effort = {};
        
        joint_state_pub_->publish(joint_state);
    }
    
    void publishOdometry(const rclcpp::Time& time)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation = tf2::toMsg(q);
        
        odom.twist.twist.linear.x = linear_vel_;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = angular_vel_;
        
        odom_pub_->publish(odom);
    }
    
    void publishTransform(const rclcpp::Time& time)
    {
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        
        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_trans.transform.rotation = tf2::toMsg(q);
        
        tf_broadcaster_->sendTransform(odom_trans);
    }
    
    double wheel_radius_;
    double wheel_base_;
    double linear_vel_ = 0.0;
    double angular_vel_ = 0.0;
    
    double x_, y_, theta_;
    double left_wheel_pos_, right_wheel_pos_;
    rclcpp::Time last_time_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MobileRobotController>());
    rclcpp::shutdown();
    return 0;
}