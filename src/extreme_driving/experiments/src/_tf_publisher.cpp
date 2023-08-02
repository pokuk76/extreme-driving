#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

class FramePublisher : public rclcpp::Node
{
    public:

    float x_p, y_p, x_o, y_o, z_o, w_o;

    FramePublisher(const std::string node_name) : Node(node_name)
    {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    ekf_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/vesc/odom", 1, [this](const nav_msgs::msg::Odometry::SharedPtr odom_in){ vescOdomCallback(odom_in); } 
        );

    timer_ = this->create_wall_timer(1ms, [this]{ timerCallback(); });

    x_p = 0.00;
    y_p = 0.00;
    x_o = 0.00;
    y_o = 0.00;
    z_o = 0.00;
    w_o = 1.00;
    
    }

    private:

    void vescOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_in)
    {
        x_p = odom_in->pose.pose.position.x;
        y_p = odom_in->pose.pose.position.y;
        x_o = odom_in->pose.pose.orientation.x;
        y_o = odom_in->pose.pose.orientation.y;
        z_o = odom_in->pose.pose.orientation.z;
        w_o = odom_in->pose.pose.orientation.w;
    }

    void timerCallback()
    {
        rclcpp::Time now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped t4;
        
        t4.header.stamp = now;
        t4.header.frame_id = "map";
        t4.child_frame_id = "odom";


        t4.transform.translation.x = 0.00;
        t4.transform.translation.y = 0.00;
        t4.transform.translation.z = 0.00;
        
        
        t4.transform.rotation.x = 0.00;
        t4.transform.rotation.y = 0.00;
        t4.transform.rotation.z = 0.00;
        t4.transform.rotation.w = 1.00;

        tf_broadcaster_->sendTransform(t4);
        
        now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped t3;


        t3.header.stamp = now;
        t3.header.frame_id = "odom";
        t3.child_frame_id = "base_link";


        t3.transform.translation.x = x_p;
        t3.transform.translation.y = y_p;
        t3.transform.translation.z = 0.00;
        
        
        t3.transform.rotation.x = x_o;
        t3.transform.rotation.y = y_o;
        t3.transform.rotation.z = z_o;
        t3.transform.rotation.w = w_o;
        
        // tf_broadcaster_->sendTransform(t3);
        
        //rclcpp::Time now = this->get_clock()->now();
        now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped t1;


        t1.header.stamp = now;
        t1.header.frame_id = "base_link";
        t1.child_frame_id = "laser";


        t1.transform.translation.x = 0.27;
        t1.transform.translation.y = 0.00;
        t1.transform.translation.z = 0.11;
        
        
        t1.transform.rotation.x = 0.00;
        t1.transform.rotation.y = 0.00;
        t1.transform.rotation.z = 0.00;
        t1.transform.rotation.w = 1.00;
        
        tf_broadcaster_->sendTransform(t1);
        
        now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped t2;


        t2.header.stamp = now;
        t2.header.frame_id = "base_link";
        t2.child_frame_id = "imu_frame";


        t2.transform.translation.x = 0.075;
        t2.transform.translation.y = 0.000;
        t2.transform.translation.z = 0.110;
        
        
        t2.transform.rotation.x = 0.00;
        t2.transform.rotation.y = 0.00;
        t2.transform.rotation.z = 0.00;
        t2.transform.rotation.w = 1.00;
        
        tf_broadcaster_->sendTransform(t2);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<FramePublisher> frame_publisher = std::make_shared<FramePublisher>("frame_publisher");

    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;

    executor->add_node(frame_publisher);

    executor->spin();

    rclcpp::shutdown();

    return 0;
}