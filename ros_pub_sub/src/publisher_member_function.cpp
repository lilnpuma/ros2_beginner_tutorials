/**
 * @file publisher_member_function.cpp
 * @author Manu Madhu Pillai (manump@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-11-16
 *
 * Apache License, Version 2.0
 * Copyright (c) 2022 Manu Madhu Pillai
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * @brief This class is used to publish a string to a topic
 * The class inherits from the rclcpp::Node class
 * The class has a publisher member function which publishes a string
 * to a topic
 * The class has a timer member function which calls the publisher member
 */
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher()
      : Node("Minimal_Publisher"), msg_("You are visitor number =") {
    publisher_ =
        this->create_publisher<std_msgs::msg::String>("custom_topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    unsigned int seed = 117817928;
    int visitor_count = rand_r(&seed) % 100;
    message.data = msg_ + " " + std::to_string(visitor_count);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string msg_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
