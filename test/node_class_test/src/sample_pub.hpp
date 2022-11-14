// make the sample pub node a class
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// SubClass/ChildClass/DerivedClass <-> SuperClass/BaseClass/ParentClass
//    {SubClass}         {SuperClass}
class SamplePub : public rclcpp::Node
                              // Node is the single point of entry for creating publishers and subscribers.
{
public:
  SamplePub();
private:
  void timerCB();
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
