#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class ClockPublisher : public rclcpp::Node {
public:
  ClockPublisher() : Node("clock_publisher"), count_(0) {
    publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&ClockPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto time = this->now();

    auto message = rosgraph_msgs::msg::Clock();
    message.clock = time;
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockPublisher>());
  rclcpp::shutdown();
  return 0;
}