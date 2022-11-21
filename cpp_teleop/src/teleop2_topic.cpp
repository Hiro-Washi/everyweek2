//
//#include <
//#include <functional.h>
#inlude <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>

#include "rclcpp/rclccp.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Map for movement keys
std::map<char, std::vector<float>> moveKeyBind
{
  {'i', {1, 0, 0, 0}},
  {'I', {1, 0, 0, 0}},
  {'j', {0, 0, 0, 1}},
  {'J', {0, 1, 0, 0}},
  {'l', {0, 0, 0, -1}},
  {'L', {0, -1, 0, 0}},
  {'k', {-1, 0, 0, 0}},
  {'K', {-1, 0, 0, 0}}
};
// Map for speed keys
std::map<char, std::vector<float>> velKeyBind
{
  {'q', {1.1, 1.1}},
  {'z', {0.9, 0.9}},
  {'w', {1.1, 1}},
  {'x', {0.9, 1}},
  {'e', {1, 1.1}},
  {'c', {1, 0.9}}
};
// Reminder message
const char* KEY_NAVI_MSG = R"(
---------------------------
Moving around:
        i 
   j    k    l

For Holonomic mode (strafing), hold down the shift key:
---------------------------
        I     
   J    K    L

---------------------------
Simple Teleoperation with arrow keys
          ↑
        ←   →
          ↓
          A
        D   C
          B
---------------------------
t   : up (+z)
b   : down (-z)
s/S : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
NOTE : Increasing or Decreasing will take affect live on the moving robot.
      Consider Stopping the robot before changing it.
CTRL-C : to quit
)";
// Init variables
float linear_vel(0.5); // Linear velocity (m/s)
float angular_vel(1.0); // Angular velocity (rad/s)
float x, y, z, th; // Forward/backward/neutral direction vars
char key(' ');

// For non-blocking keyboard inputs
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

// Function to check speed is in the range or not
// Used to linearly increase/decrease the speed
float vel_check(float curr, bool decrease = false){
  if (decrease)
    curr = (curr >= -0.95) ? curr-0.05 : -1;
  else
    curr = (curr <= 0.95) ? curr+0.05 : 1;
  return curr;
}

// Linear vel for arrow keys
float Lvel(char key, float x){
  if(key=='A')
    return vel_check(x,false);
  if(key=='B')
    return vel_check(x,true);
  return 0;
}
// Angular vel for arrow keys
float Avel(char key, float th){
  if(key=='C')
    return vel_check(th,true);
  if(key=='D')
    return vel_check(th,false);
  return 0;
}


int main(int argc, char** argv){
  rclcpp::init(argc,argv);
  auto  = rclcpp::Node::make_shared("teleop2_pub"); //-----------------
  auto twist_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  geometry_msgs::msg::Twist twist;
  printf("%s", KEY_NAVI_MSG);
  printf("\nNow top Speed is %f and turn is %f | Last command: \n", linear_vel, angular_vel);

  while(rclcpp::ok()){
    // get the pressed key
    key = getch();

    //'A' and 'B' represent the Up and Down arrow keys consecutively 
    if(key=='A'||key=='B'){
      x = Lvel(key, x);
      y = 0.0;
      z = 0.0;
      printf("\rCurrent: Speed %f\tturn %f | Last command: %c   ", linear_vel*x, angular_vel*th, key);
    }

    //'C' and 'D' represent the Right and Left arrow keys consecutively 
    else if(key=='C'||key=='D'){
      th = Avel(key,th);
      y = 0.0;
      z = 0.0;
      printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", linear_vel*x, angular_vel*th, key);
    }

    else if (moveKeyBind.count(key) == 1)
    {
      // Grab the direction data
      x = moveKeyBind[key][0];
      y = moveKeyBind[key][1];
      z = moveKeyBind[key][2];
      th = moveKeyBind[key][3];

      printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", linear_vel, angular_vel, key);
    } 
    // Otherwise if it corresponds to a key in velKeyBind
    else if (velKeyBind.count(key) == 1)
    {
      // Grab the speed data
      linear_vel = linear_vel * velKeyBind[key][0];
      angular_vel = angular_vel * velKeyBind[key][1];

      printf("\nNow top Speed is %f and turn is %f | Last command: %c \n\t\tCurrent speed might be affected\n", linear_vel, angular_vel, key);
    }

    // Otherwise, set the robot to stop
    else
    { if (key=='s'||key=='S'){
      x = 0;
      y = 0;
      z = 0;
      th = 0;
      printf("\n\t\tRobot Stopped..!! \n");
      printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", linear_vel*x, angular_vel*th, key);
    }
      // If ctrl-C (^C) was pressed, terminate the program
      else if (key == '\x03')
      {
        printf("\n\n    ☺  Give it a Star :: https://github.com/1at7/teleop_cpp_ros2 ☺ \n\n");
        break;
      }
      else
        printf("\rCurrent: linear_vel %f\tturn %f | Invalid command! %c", linear_vel*x, angular_vel*th, key);
    }

    // Update the Twist message
    twist.linear.x = x * linear_vel;
    twist.linear.y = y * linear_vel;
    twist.linear.z = z * linear_vel;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th * angular_vel;
    twist_pub->publish(twist);
    //Create a default single-threaded executor and execute any immediately available work.
    rclcpp::spin_some(node);  
    //spin() : Create a default single-threaded executor and spin the specified node.
  }
  return 0;

}
