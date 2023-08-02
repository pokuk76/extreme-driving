#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("tf_publisher")
  {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    timer_ = this->create_wall_timer(1ms, std::bind(&FramePublisher::timer_callback, this));
  }

private:

  void timer_callback()
  {
    rclcpp::Time now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped t;


    t.header.stamp = now;
    t.header.frame_id = "base_link";
    t.child_frame_id = "laser";


    t.transform.translation.x = 0.27;
    t.transform.translation.y = 0.00;
    t.transform.translation.z = 0.11;
    
    
    t.transform.rotation.x = 0.00;
    t.transform.rotation.y = 0.00;
    t.transform.rotation.z = 0.00;
    t.transform.rotation.w = 1.00;
    
    tf_broadcaster_->sendTransform(t);
    
    rclcpp::Time now2 = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped t2;
    
    t2.header.stamp = now2;
    t2.header.frame_id = "base_link";
    t2.child_frame_id = "odom";


    t2.transform.translation.x = 0.00;
    t2.transform.translation.y = 0.00;
    t2.transform.translation.z = 0.00;
    
    
    t2.transform.rotation.x = 0.00;
    t2.transform.rotation.y = 0.00;
    t2.transform.rotation.z = 0.00;
    t2.transform.rotation.w = 1.00;

    tf_broadcaster_->sendTransform(t2);


    rclcpp::Time now3 = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped t3;


    t3.header.stamp = now3;
    t3.header.frame_id = "base_link";
    t3.child_frame_id = "imu_frame";


    t3.transform.translation.x = 0.075;
    t3.transform.translation.y = 0.000;
    t3.transform.translation.z = 0.110;
    
    
    t3.transform.rotation.x = 0.00;
    t3.transform.rotation.y = 0.00;
    t3.transform.rotation.z = 0.00;
    t3.transform.rotation.w = 1.00;
    
    tf_broadcaster_->sendTransform(t3);

  }
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
}
