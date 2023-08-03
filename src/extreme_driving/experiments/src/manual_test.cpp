#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <thread>
#include <functional>

// For timer implementations (+ <functional>)
#include <chrono>
#include <boost/system/error_code.hpp>
#include <boost/asio.hpp>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std;
using namespace Eigen;

# define PI 3.14159265358979323846

using namespace std::chrono_literals;
using Clock = std::chrono::high_resolution_clock;
using Callback = std::function<bool()>;
using boost::system::error_code;

struct interval_timer
{
    interval_timer();

    interval_timer(boost::asio::io_context& io, Clock::duration i, Callback cb) : interval(i), callback(cb), timer(io) {}

    void run()
    {
        timer.expires_from_now();
        timer.async_wait(
          [=](boost::system::error_code ec)
          {
            if (!ec && callback()) run();
          }
        );
    }

    void stop()
    {
        timer.cancel();
    }

    private:
      Clock::duration const interval;
      Callback callback;
      boost::asio::high_resolution_timer timer;
};

class ManualDrive : public rclcpp::Node
{
	public:
        int interrupt_time, thread_count;
        double vk, steering_raw;
        bool enableController;
        float t_delay_;
        boost::asio::io_context io;

        interval_timer t {
            io,
            200000us,
            [&]()
            {
                thread_count++;
                return true;
            }
        };
        
        ManualDrive() : Node("ManualDrive") {
            declare_parameter("interrupt_time", 1000);
            declare_parameter("speed", 0.3);
            declare_parameter("steering_angle", 0.0);


            t_delay_ = ((float) get_parameter("interrupt_time").as_int())*1e-6;
            vk = get_parameter("speed").as_double();
            steering_raw = get_parameter("steering_angle").as_double();


            // Teleop controller
            ackermann_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);
            joystick_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, [this](sensor_msgs::msg::Joy::SharedPtr msg){ process_joystick(msg); });
            timer_ = this->create_wall_timer(20ms, [this]{ timer_callback(); });


            // Drifting maneuvers
            // subscriber_ = this->create_subscription<std_msgs::msg::Int32>("/icp_state", 1, [this](std_msgs::msg::Int32::SharedPtr msg){ msg_callback(msg); });
            misc_pub = this->create_publisher<std_msgs::msg::Int32>("/interrupt_data", 1);
            // drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);

            // TODO: Change frequency back to 25ms once we integrate the LiDAR
            timer_ = this->create_wall_timer(25ms, [this]{ timer_callback(); });

            enableController = true;
        }

	private:


        void teleop_timer_callback()
        {
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.drive.steering_angle = 0.50*axs.at(0);
            drive_msg.drive.speed = (2.25*(axs.at(2) - axs.at(5)))/2; 
            ackermann_publisher->publish(drive_msg);
        }
        
        void process_joystick(const sensor_msgs::msg::Joy::SharedPtr joy_in)
        {
            axs = joy_in.get()->axes;
        }

        
        void msg_callback(const std_msgs::msg::Int32::SharedPtr state_msg) {
            // int data_icp = state_msg.get()->data;

            // t.clear = true;

            std_msgs::msg::Int32 rising;

            rising.data = thread_count;
            misc_pub->publish(rising);
        }

        
        void timer_callback() {
            ackermann_msgs::msg::AckermannDriveStamped drive_msg;

            drive_msg.drive.steering_angle = steering_raw*int(enableController);
            drive_msg.drive.speed = vk*int(enableController);

            drive_pub_->publish(drive_msg);
        }

	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr misc_pub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscription;

    // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
	
	rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualDrive>());
  rclcpp::shutdown();
  return 0;
}
