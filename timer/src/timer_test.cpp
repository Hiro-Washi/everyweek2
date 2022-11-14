// rclに基づいた新しいタイマー
// c++のchronoを使用
#include <rclcpp/rclcpp.hpp>
#include <chrono>

// ~ = ROS node pointer
// SharedPtr - 参照カウンタを持ち、リソースが共有できるスマートポインタ
// オブジェクトが生成されると参照カウンタは1にセット。共有が増えるたびに、カウンタが１増やされ、共有が減るたびに-1。カウンタが0になった時点でリソース開放.参照カウントを持つため、メモリの使用量や管理コストが上がるため、性能面で生のポインタと比較すると、劣る。
rclcpp::Node::SharedPtr node = nullptr;

int main(int argc, char * argv[]){
  using namespace std::chrono_literals;

  rclcpp::init(argc,argv); //init 
  node = rclcpp::Node::make_shared("timer_test"); // Create a node

  // chrono sysytem - 単位月の時間を設定できる
  rclcpp::WallRate loop_rate(500ms); // loop_rate

  // Rate system timer
  for ( int i=0 ; i<3 ; i++ ){
    RCLCPP_INFO(node->get_logger(), "loop;%d", i); // std Output
    loop_rate.sleep(); // ()の中はスリーブ時間の設定不可
  }

  // rclcpp::
  for (int i=0 ; i<3 ; i++ ){
    RCLCPP_INFO(node->get_logger(), "loop:%d", i);
    rclcpp::sleep_for(500ms);
  }

  // Timer system | create_wall_timer( ~ )
  auto timer1 = node->create_wall_timer(
    1s, // First arg is exec period
    // Sec srg is lambda func (static process)
    [](){
      RCLCPP_INFO(node->get_logger(), "node_loop");
    }
  );
  rclcpp::spin(node); // execute node for the static processing


  rclcpp::shutdown();
  return 0;
}
