#include <memory> // to use smart pointer
#include <chrono> // to do Processing time measurement
#include <string> // to use string class
                  //  and include these header, <initializer_list>, <compre>
#include "rclcpp/rclcpp.hpp"
#incluce "std_msgs/msg/string.hpp"

using namespace std::chrono_literals; // Nice literal operator

// Create TelePub Node Class, inheriting from rclcpp::Node
class TelePub : public rclcpp::Node {
                // 継承
  public:
    TelePub()
      //rcl::Node::Node( const std::string & node_name,
      //                 const std::string & namespace_ = "", 
      //                                     (↑Namespace of the node)
      //                 	bool 	use_intra_process_comms = false )
    : Node("teleop2"), count_(0) // Message count inited to 0
    {
      //     rclcpp::Node                            (msg, queue_size)
      pub_ = this->create_publisher<std::msg::String>("teleop2_pub_msg", 1);
      //       rclcpp::Node           (period, CB/Static process,                target group CB is in )
      timer_ = this->create_wall_timer( 200ms, std::bind(&TelePub::timerCB, this));
      // std::bind links arg1 obj and arg2.    = timerCB(TelePub:~)???      = address of TelePub???
      // and create/return the func
    }
  private:
    void timerCB()
    {
      auto timer_msg = std_msgs::msg::String();
      timer_msg.data = " TEST MSG... " + std::to_string(count_++); // count_++
      RCLCPP_INFO(this->get_logger(), "Pub: '%s'", timer_msg.data.c_str()); 
      　　　　　　　　　　　　　　　　　　　　　　　　　　//c_str(): Char型文字列を取得する
      pub_->publish(timer_msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher::<std_msgs::msg::String>::SharePtr pub_;
    size_t count_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TelePub>()); //--------------
  rclcpp::shutdown();
  return 0;
}
