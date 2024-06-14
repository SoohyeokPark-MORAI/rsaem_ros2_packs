#ifndef RSAEMBOT_CORE_CONFIG_H_
#define RSAEMBOT_CORE_CONFIG_H_

#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "rsaem_msgctl/msg/msg_ctl.hpp"


#include <math.h>


#define LEFT 0
#define RIGHT 1


//#define IMU_FUNCTION

//#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

/*-------------aibot----------------*/
#include <pthread.h>
#include <signal.h>
#include "cansocket.hpp"

/*-------------aibot----------------*/

#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define CONTROL_MOTOR_TIMEOUT                  500  //ms
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   //hz


#define WHEEL_NUM                        2

#define LEFT                             0
#define RIGHT                            1

#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define TICK2RAD                         0.001498851  // 0.085877863[deg] * 3.14159265359 / 180 = 0.001533981f


uint32_t current_offset;



/*******************************************************************************
* aibot
*******************************************************************************/

#define AIBOT_GO 0
#define AIBOT_STOP 1

#define MSG_AIBOT_WHEEL 0x01
#define MSG_AIBOT_ENCODER 0x11
#define MSG_AIBOT_ENC_RESET 0x09

#define PI_M 3.141592654

#define CAN_ENC_OFFSET 2147483647

#define lengthBetweenTwoWheels 0.21
#define wheelRadius 0.04

//#define RPR_M (131*16*2)
#define RPR_M (100*13*2)

//#TICK4RAD ((360.0/RPR_M)*(180.0/PI_M))
//#define RPR_M (131*16*2)

#define DistancePerCount ((PI_M * wheelRadius * 2) / RPR_M) // (2*PI*r)/ppr

uint8_t JOINT_NUM = 2;

int32_t PrevLeftEncoder;
int32_t PrevRightEncoder;
int dirL, dirR;

int sendCanTx(int speed, int dir, int heading, int aibotgo);
void *recvThread(void* data);

float motorLeft, motorRight;

float steering_gain_value, steering_dgain_value;


int RunningMode_F;
int RunningMode_B;

int SaftyMode_F;
int SaftyMode_B;

class RSaemBot : public rclcpp::Node
{
public:
  RSaemBot();
  ~RSaemBot();

private:
  // ROS time
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time prev_update_time_;
  
  rclcpp::Time current_time;
  rclcpp::Time last_time;

  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // ROS topic publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_; //WHEEL
  //rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


  // ROS topic subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<rsaem_msgctl::msg::MsgCtl>::SharedPtr msgctl_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reset_sub;
  

  nav_msgs::msg::Odometry odom_;
  

	

  double wheel_speed_cmd_[2];
  double goal_linear_velocity_;
  double goal_angular_velocity_;
  double cmd_vel_timeout_;

///
  double wheels_separation_; 
  double wheels_radius_;
  
  bool init_encoder;
  
  std::string frame_id_of_odometry_;
  std::string child_frame_id_of_odometry_;

  bool use_imu_;
  bool publish_tf_;
  
  
  
  ///
  sensor_msgs::msg::JointState joint_states_; 
  
  std::array<double, 2> last_diff_position_;
  std::array<double, 2> last_position_;
  std::array<double, 2> last_delta_position_;
  
  std::array<double, 2> diff_joint_positions_;
  std::array<double, 2> last_joint_positions;

  double imu_angle_;

  std::array<double, 3> robot_pose_;
  std::array<double, 3> robot_vel_;
  
//// 
  void joint_state_enc_callback(void);
  void update_joint_state_publish(const rclcpp::Duration & duration);
  void odom_publish(const rclcpp::Time & now);
  bool calculate_odometry(const rclcpp::Duration & duration);
  void init_odom(void); 
  
	//////////////
  // Function prototypes
  void init_parameters();
  //void init_variables();
  
  
  void rsaem_msgctl_callback(const rsaem_msgctl::msg::MsgCtl::SharedPtr msg);
  void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
  void resetCallback(const std_msgs::msg::String::SharedPtr reset_msg); 
    //void update_callback();
  //bool update_odometry(const rclcpp::Duration & diff_time);
  //void update_joint_state();
  //void update_tf(geometry_msgs::msg::TransformStamped & odom_tf);
};

#endif // RSAEMBOT_CORE_CONFIG_H_

