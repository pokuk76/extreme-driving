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
        timer.expires_from_now(interval);
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

class Interrupter : public rclcpp::Node
{
	public:
    int interrupt_time, thread_count;
    float t_delay_;
    boost::asio::io_context io;

    interval_timer t {
        io,
        200000us,
        [&]()
        {
            *ptr ^= 1;
            thread_count++;
            return true;
        }
    };
	
	Interrupter() : Node("Interrupter")
	{
        declare_parameter("interrupt_time", 1000);
        t_delay_ = ((float) get_parameter("interrupt_time").as_int())*1e-6;
        thread_count = 0;
        // *ptr = 0;

        // t = interval_timer 
        // (
        //     io,
        //     200000us,
        //     [&]()
        //     {
        //         *ptr ^= 1;
        //         thread_count++;
        //         return true;
        //     }
        // );



        // subscriber_ = this->create_subscription<std_msgs::msg::Int32>("/icp_state", 1, [this](std_msgs::msg::Int32::SharedPtr msg){ msg_callback(msg); });
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/interrupt_data", 1);
	}

	private:

    void msg_callback(const std_msgs::msg::Int32::SharedPtr state_msg)
    {
        int data_icp = state_msg.get()->data;

        // t.clear = true;

        std_msgs::msg::Int32 rising;


        rising.data = thread_count;
        publisher_ -> publish(rising);
    }

	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
	
	rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Interrupter>());
  rclcpp::shutdown();
  return 0;
}