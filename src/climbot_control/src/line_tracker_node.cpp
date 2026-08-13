#include <chrono>
#include <memory>
#include "climbot_control/line_tracker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;
class Node : public rclcpp::Node {
public: Node() : rclcpp::Node("line_tracker") {
    start_ = {declare_parameter("start_x", 0.0), declare_parameter("start_y", 0.0)};
    end_ = {declare_parameter("end_x", 1.0), declare_parameter("end_y", 0.0)};
    cruise_speed_ = declare_parameter("cruise_speed", 0.15);
    cross_gain_ = declare_parameter("cross_gain", 1.0);
    heading_gain_ = declare_parameter("heading_gain", 2.0);
    pub_=create_publisher<geometry_msgs::msg::Twist>("/control/cmd_vel", 10);
    sub_=create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10,[this](nav_msgs::msg::Odometry::SharedPtr m){pose_={m->pose.pose.position.x,m->pose.pose.position.y,2*std::atan2(m->pose.pose.orientation.z,m->pose.pose.orientation.w)}; have_=true;});
    timer_=create_wall_timer(20ms,[this](){if(!have_) return; auto c=climbot_control::trackLine(start_,end_,pose_,cruise_speed_,cross_gain_,heading_gain_,{}); geometry_msgs::msg::Twist msg; msg.linear.x=c.linear; msg.angular.z=c.angular; pub_->publish(msg);});}
private: bool have_{false}; double cruise_speed_{},cross_gain_{},heading_gain_{}; climbot_control::Point2 start_{},end_{}; climbot_control::Pose2 pose_{}; rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_; rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_; rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Node>());rclcpp::shutdown();}
