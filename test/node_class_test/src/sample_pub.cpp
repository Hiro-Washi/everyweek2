#include "node_class_test/sample_pub.hpp"

SamplePub::SamplePub() : Node("sample_pub") {
  pub_ = this->create_publisher<std_msgs::msg::String>("greet", 1);

  using namespace std::chrono_literals;
  timer_ = this->create_wall_timer(1s, 
                                   std::bind(&SamplePub::timerCB, 
                                   this));
