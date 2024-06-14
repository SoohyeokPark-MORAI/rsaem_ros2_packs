// Copyright 2024 JetsonAI CO., LTD.
//
// Author: Kate Kim

#include <cstdio>
   
#include "rsaem_main.hpp"
#include <string>
#include <sstream>

#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include "cansocket.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>

#include "rclcpp/rclcpp.hpp"


//#include "odometry.hpp"

//#include <memory>
#include "diff_drive_controller.hpp"

using namespace std::chrono_literals;

using namespace std;

static volatile bool gRun = true;

void init_can(void);

void rsaembot_setup(void);
//void aibot_loop(void);
void *rsaembot_loop(void* data);
void close_rsaembot(void);

void sig_int_handler(int sig);
void set_signal(void);
int sendInitEnc(void);
int sendCanTx2(float speed, int dir, float angle, int aibotgo);
void motor_contol_test(void);
void motor_contol(float speedCmd, float headingCmd, uint32_t dirCmd, uint32_t stopCmd);
int recvCan2Odom(int32_t * LeftEncoder, int32_t *RightEncoder);
bool setup_end = false;

using std::placeholders::_1;

RSaemBot::RSaemBot()
: Node("rsaem_node")
{
  init_parameters();

  init_odom();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  
//////////////////////

  // Initialise subscribers
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos, std::bind(&RSaemBot::command_velocity_callback, this, _1));

  msgctl_sub_ = this->create_subscription<rsaem_msgctl::msg::MsgCtl>(
    "MotorCtl", qos, std::bind(&RSaemBot::rsaem_msgctl_callback, this, std::placeholders::_1));

  reset_sub = this->create_subscription<std_msgs::msg::String>(
    "reset", qos, std::bind(&RSaemBot::resetCallback, this, std::placeholders::_1));
    
//////////////////////       
  // Initialise publishers
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  
  joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos); //WHEEL

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  /************************************************************
  ** initialise ROS timers
  ************************************************************/
  std::chrono::milliseconds timeout = std::chrono::milliseconds(30);
  update_timer_ = this->create_wall_timer(timeout, std::bind(&RSaemBot::joint_state_enc_callback, this)); 

  RCLCPP_INFO(this->get_logger(), "R-SaemBot node has been initialised");
}


RSaemBot::~RSaemBot()
{
  RCLCPP_INFO(this->get_logger(), "RSaemBot node has been terminated");
}

/********************************************************************************
** Init functions
********************************************************************************/
void RSaemBot::init_parameters()
{
  // Declare parameters that may be set on this node
  RCLCPP_INFO(this->get_logger(), "[init_parameters]");

  this->declare_parameter("odometry.frame_id");
  this->declare_parameter("odometry.child_frame_id");
  this->declare_parameter("odometry.publish_tf");

  this->get_parameter_or<bool>(
    "odometry.publish_tf", publish_tf_, false);

  this->get_parameter_or<std::string>(
    "odometry.frame_id", frame_id_of_odometry_, std::string("odom"));

  this->get_parameter_or<std::string>(
    "odometry.child_frame_id", child_frame_id_of_odometry_, std::string("base_footprint"));
  
  frame_id_of_odometry_ = "odom";
  child_frame_id_of_odometry_ = "base_footprint";
  use_imu_ = false;
  publish_tf_ = true;
   
  RCLCPP_INFO(this->get_logger(), "publish_tf_:%d", publish_tf_ ); 
  RCLCPP_INFO(this->get_logger(), "frame_id_of_odometry_:%s", frame_id_of_odometry_.c_str() ); 
  RCLCPP_INFO(this->get_logger(), "child_frame_id_of_odometry_:%s", child_frame_id_of_odometry_.c_str() ); 
  wheels_separation_ = 0.09; // 0.160;
  wheels_radius_ = 0.04;  //0.033;  
  
}


void RSaemBot::update_joint_state_publish(const rclcpp::Duration & duration)
{
  //static std::array<double, 2> last_joint_positions = {0.0f, 0.0f};
  //RCLCPP_INFO(this->get_logger(), "[update_joint_state_publish]");

  static std::array<double, 2> position, delta_position, velocity;
  double step_time = duration.seconds();
  //RCLCPP_INFO(this->get_logger(), "step_time:%d",step_time);

  int32_t LeftEncoder = 0;
  int32_t RightEncoder = 0;
   
  recvCan2Odom(&LeftEncoder, &RightEncoder);
   
  position[0] = LeftEncoder;
  position[1] = RightEncoder;
    
    delta_position[0] = position[0] - last_position_[0];
    delta_position[1] = position[1] - last_position_[1];
    
	if(abs(delta_position[0]) > 3000)
		delta_position[0] = last_delta_position_[0];
	if(abs(delta_position[1]) > 3000)
		delta_position[1] = last_delta_position_[1];   
		
    last_delta_position_ =  delta_position;
    last_position_ = position;
    
    //RCLCPP_INFO(this->get_logger(), "delta_position[0]: %f delta_position[1]: %f", delta_position[0], delta_position[1]);
     
  velocity[0] = (delta_position[0] * DistancePerCount) / step_time;
  velocity[1] = (delta_position[1] * DistancePerCount) / step_time;

  //RCLCPP_INFO(this->get_logger(), "velocity[0]: %f velocity[1]: %f", velocity[0], velocity[1]);

  joint_states_.position[0] = TICK2RAD * last_diff_position_[0];
  joint_states_.position[1] = TICK2RAD * last_diff_position_[1];
  joint_states_.velocity[0] = velocity[0];
  joint_states_.velocity[1] = velocity[1];

  
  joint_states_.header.stamp = current_time;
  joint_states_pub_->publish(joint_states_);  

  ////////////////////////    
  
  last_diff_position_[0] += delta_position[0];
  last_diff_position_[1] += delta_position[1];    
  
  diff_joint_positions_[0] = joint_states_.position[0] - last_joint_positions[0];
  diff_joint_positions_[1] = joint_states_.position[1] - last_joint_positions[1];

  last_joint_positions[0] = joint_states_.position[0];
  last_joint_positions[1] = joint_states_.position[1];  
  
}



/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/


/********************************************************************************
** Update functions
********************************************************************************/
void RSaemBot::joint_state_enc_callback(void)
{
  //RCLCPP_INFO(this->get_logger(), "[joint_state_enc_callback]");
  
  current_time = this->now();  
  rclcpp::Duration duration((current_time - last_time).nanoseconds());
  
  update_joint_state_publish(duration);

  calculate_odometry(duration);
  odom_publish(current_time);

  last_time = current_time;

}

void RSaemBot::init_odom(void)
{
    RCLCPP_INFO(this->get_logger(), "[init_odom]");
    sendInitEnc();

    init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    robot_pose_[index] = 0.0;
    robot_vel_[index]  = 0.0;
  }

  last_diff_position_[0] = 0.0; // left
  last_diff_position_[1] = 0.0; // right

  last_position_[0] = 0.0; // left
  last_position_[1] = 0.0; // right
   
  last_delta_position_[0] = 0.0; // left
  last_delta_position_[1] = 0.0; // right
     
  diff_joint_positions_[0] = 0.0; // left
  diff_joint_positions_[1] = 0.0; // right

  last_joint_positions[0] = 0.0; // left
  last_joint_positions[1] = 0.0; // right
  
  last_time = this->now();
  /////////////////////
//  joint_states_.name.push_back("wheel_left_joint");
//  joint_states_.name.push_back("wheel_right_joint");
  joint_states_.header.frame_id = "base_link";
  joint_states_.name.push_back("left_wheel_hinge");
  joint_states_.name.push_back("right_wheel_hinge");  
  joint_states_.position.resize(2, 0.0);
  joint_states_.velocity.resize(2, 0.0);
  joint_states_.effort.resize(2, 0.0);
}



bool RSaemBot::calculate_odometry(const rclcpp::Duration & duration)
{
  // RCLCPP_INFO(this->get_logger(), "[calculate_odometry]");
  // rotation value of wheel [rad]
  double wheel_l = diff_joint_positions_[0];
  double wheel_r = diff_joint_positions_[1];

  double delta_s = 0.0;
  double delta_theta = 0.0;

  double theta = 0.0;
  static double last_theta = 0.0;

  double v = 0.0;
  double w = 0.0;

  double step_time = duration.seconds();
  //RCLCPP_INFO(this->get_logger(), "step_time:%d", step_time );

  if (step_time == 0.0) {
    return false;
  }

  if (std::isnan(wheel_l)) {
    wheel_l = 0.0;
  }

  if (std::isnan(wheel_r)) {
    wheel_r = 0.0;
  }
  
  delta_s = wheels_radius_ * (wheel_r + wheel_l) / 2.0;

  if (use_imu_) {
    theta = imu_angle_;
    delta_theta = theta - last_theta;
  } else {
    theta = wheels_radius_ * (wheel_r - wheel_l) / wheels_separation_;
    delta_theta = theta;
  }

  // compute odometric pose
  robot_pose_[0] += delta_s * cos(robot_pose_[2] + (delta_theta / 2.0));
  robot_pose_[1] += delta_s * sin(robot_pose_[2] + (delta_theta / 2.0));
  robot_pose_[2] += delta_theta;

  RCLCPP_DEBUG(this->get_logger(), "x : %f, y : %f", robot_pose_[0], robot_pose_[1]);

  // compute odometric instantaneouse velocity
  v = delta_s / step_time;
  w = delta_theta / step_time;

  robot_vel_[0] = v;
  robot_vel_[1] = 0.0;
  robot_vel_[2] = w;

  last_theta = theta;
  return true;
}

void RSaemBot::odom_publish(const rclcpp::Time & now)
{
  //RCLCPP_INFO(this->get_logger(), "[odom_publish]");
  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

  odom_msg->header.frame_id = frame_id_of_odometry_;
  odom_msg->child_frame_id = child_frame_id_of_odometry_;
  odom_msg->header.stamp = current_time;

  odom_msg->pose.pose.position.x = robot_pose_[0];
  odom_msg->pose.pose.position.y = robot_pose_[1];
  odom_msg->pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, robot_pose_[2]);

  odom_msg->pose.pose.orientation.x = q.x();
  odom_msg->pose.pose.orientation.y = q.y();
  odom_msg->pose.pose.orientation.z = q.z();
  odom_msg->pose.pose.orientation.w = q.w();

  odom_msg->twist.twist.linear.x = robot_vel_[0];
  odom_msg->twist.twist.angular.z = robot_vel_[2];

  geometry_msgs::msg::TransformStamped odom_tf;

  odom_tf.transform.translation.x = odom_msg->pose.pose.position.x;
  odom_tf.transform.translation.y = odom_msg->pose.pose.position.y;
  odom_tf.transform.translation.z = odom_msg->pose.pose.position.z;
  odom_tf.transform.rotation = odom_msg->pose.pose.orientation;

  odom_tf.header.frame_id = frame_id_of_odometry_;
  odom_tf.child_frame_id = child_frame_id_of_odometry_;
  odom_tf.header.stamp = now;

  odom_pub_->publish(std::move(odom_msg));
  //RCLCPP_INFO(this->get_logger(), "[odom_pub_->publish]");

  if (publish_tf_) {
    //RCLCPP_INFO(this->get_logger(), "[tf_broadcaster_]");
    tf_broadcaster_->sendTransform(odom_tf);
  }

}

void RSaemBot::rsaem_msgctl_callback(const rsaem_msgctl::msg::MsgCtl::SharedPtr msg) 
{ 
  RCLCPP_INFO(this->get_logger(), "recieve speedCmd:%0.2f headingCmd:%0.2f dir_cmd:%d stop_cmd:%d", 
     msg->speed_cmd, msg->heading_cmd, msg->dir_cmd , msg->stop_cmd );

  motor_contol(msg->speed_cmd, msg->heading_cmd, msg->dir_cmd, msg->stop_cmd);

}
void RSaemBot::command_velocity_callback(
  const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg)
{
      RCLCPP_INFO(this->get_logger(), "[command_velocity_callback]");
      float speedCmd = 0;
      float headingCmd0 = 0;
        
      speedCmd  = cmd_vel_msg->linear.x;
      headingCmd0 = cmd_vel_msg->angular.z;  
      
      RCLCPP_INFO(this->get_logger(), "recieve speedCmd:%0.2f  headingCmd0:%0.2f ", 
         speedCmd, headingCmd0);
 
      uint32_t dirCmd = 0;
      uint32_t stopCmd = 0;

      //float headingCmd = -headingCmd0; //20230629 mybot test
      float headingCmd = headingCmd0; //20230629 turtle

      if(speedCmd > 0) {
	      dirCmd = 0; //1;
      } else if (speedCmd < 0) {
	      dirCmd = 1; //0;
      } 
      speedCmd = abs(speedCmd);
      if (speedCmd == 0 && headingCmd==0) {
	      stopCmd = 1;
      } 
      //printf("speedCmd:%0.2f headingCmd:%0.2f dirCmd:%d stopCmd:%d\n", speedCmd, headingCmd, dirCmd, stopCmd);	   
      sendCanTx2( speedCmd, dirCmd, headingCmd, stopCmd);

/*
  last_cmd_vel_time_ = this->now();

  goal_linear_velocity_ = cmd_vel_msg->linear.x;
  goal_angular_velocity_ = cmd_vel_msg->angular.z;

  wheel_speed_cmd_[LEFT] = goal_linear_velocity_ - (goal_angular_velocity_ * wheel_seperation_ / 2);
  wheel_speed_cmd_[RIGHT] = goal_linear_velocity_ + \
    (goal_angular_velocity_ * wheel_seperation_ / 2);
*/
    
}



void RSaemBot::resetCallback(const std_msgs::msg::String::SharedPtr reset_msg)
{
  //(void)(reset_msg);

  init_odom();
  motor_contol(0,0,0,1);

	RCLCPP_INFO(this->get_logger(), "Reset Odometry");
 
}

void rsaembot_setup(void)
{
  init_can();

  setup_end = true;
}


void motor_contol(float speedCmd, float headingCmd, uint32_t dirCmd, uint32_t stopCmd)
{
    if(speedCmd > 0) {
	  dirCmd = 0; 
    } else if (speedCmd < 0) {
	  dirCmd = 1; //0;
    } 
    speedCmd = abs(speedCmd);
    if (speedCmd == 0 && headingCmd==0) {
      stopCmd = 1;
    } 
  printf("speedCmd:%0.2f headingCmd:%0.2f dirCmd:%d stopCmd:%d\n", speedCmd, headingCmd, dirCmd, stopCmd);	   
  sendCanTx2( speedCmd, dirCmd, headingCmd, stopCmd);

}

/*******************************************************************************
* Loop function
*******************************************************************************/

void *rsaembot_loop(void* data)
{
  printf("<rsaembot_loop>\n");
  rclcpp::Rate loop_rate(30);

    while (gRun && rclcpp::ok())
    {
        printf("rsaembot_loop \n");
        //motor_contol_test();
        loop_rate.sleep();              
    }
    printf("end loop thread..\n");
    motor_contol(0,0,0,1);
    sleep(1);
    close_port();

}

void set_signal(void)
{
  struct sigaction action = {};
  action.sa_handler = sig_int_handler;

  sigaction(SIGHUP, &action, NULL);  // controlling terminal closed, Ctrl-D
  sigaction(SIGINT, &action, NULL);  // Ctrl-C
  sigaction(SIGQUIT, &action, NULL); // Ctrl-\, clean quit with core dump
  sigaction(SIGABRT, &action, NULL); // abort() called.
  sigaction(SIGTERM, &action, NULL); // kill command	
}

void sig_int_handler(int sig)
{
    
    (void)sig;
    gRun = false;
    motor_contol(0,0,0,1);
    close_port();
}

void init_can(void)
{
  if(open_port("can0") < 0)
  {
    printf("open can error:");
    exit(0);

  }
  printf("can0 opened.\n");	
}

void close_rsaembot(void)
{
  printf("close_rsaembot\n");
  close_port();	
}

#define MotorMin 130 //55
#define MotorMax 255
#define InputMin 0
#define InputMax 1.0

int motor_map( float x ) 
{
    if(x >= 1) x = 1;

    int out_min = MotorMin;
    int out_max = MotorMax;
    float in_min= InputMin;
    float in_max= InputMax;
    int ret =(x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min ;
    // For debugging:
    //printf("MAPPED %f to: %d\n", x, ret);
    return ret ;
}

float getmin(float f1, float f2)
{
    if(f1 <= f2)
       return f1;
    else
       return f2;
}

float getmax(float f1, float f2)
{
    if(f1 >= f2)
       return f1;
    else
       return f2;
}

int sendInitEnc(void) 
{
	struct can_frame canmsg, res;
    memset((void*)&canmsg, 0x00, sizeof(struct can_frame));
    canmsg.can_id = MSG_AIBOT_ENC_RESET; 

    if (send_port(&canmsg)) {
        //printf("message sent !!\n");
    } else {
        printf("send fail sendInitEnc\n");
    } 
    return 0;	
}


int sendCanTx2(float speed, int dir, float angle, int aibotgo)
{
    struct can_frame canmsg, res;
    static float pr_speed2, pr_angle;
    static int pr_dir2, pr_aibotgo2;
    int speedL, speedR;

    if(pr_speed2==speed && pr_dir2==dir && pr_angle==angle && pr_aibotgo2==aibotgo) 
    {
        return -1;
    }
    else 
    {
      pr_speed2=speed;
      pr_dir2=dir;
      pr_angle=angle;
      pr_aibotgo2=aibotgo;
    }
	
	/*if(aibotgo==1) {
		printf("<RunningMode:(%d)(%d)(%d)(%d) >>>>>>>> STOP!!\n", 
		                    RunningMode_F, RunningMode_B, SaftyMode_F, SaftyMode_B);
	}*/
	
    printf("[sendCanTx2] speed:%0.2f angle:%0.2f dir:%d aibotgo:%d\n", speed, angle, dir, aibotgo);
    float left_x = 0;
    float right_x = 0;
    dirL = dirR = dir;
    if(speed == 0 && angle != 0) {
        if(angle > 0) { //right side 
            printf("Turn right side");
            dirL = 0;
            dirR = 1;	  
        } 
        else if(angle < 0) { //left side
            printf("Turn left side");
            dirL = 1;
            dirR = 0;	  
        }
        left_x = right_x = 0.1;
    }

    if(speed > 0) 
	{   
		static const float steering_dgain = 0.1;
		static const float speed_gain = 1.0;	
		static int angle_last;
        float pid = angle * steering_dgain + (angle-angle_last) * steering_dgain;
        angle_last = angle;
	
        float speed_value = speed * speed_gain;
        float steering_value = pid;
    
        left_x = getmax(getmin(speed_value + steering_value, 1.0), 0.0);
        right_x = getmax(getmin(speed_value - steering_value, 1.0), 0.0);
    }

    left_x = left_x * motorLeft;
    right_x = right_x * motorRight;
    printf("motorLeft %f motorRight %f left_x %f right_x %f\n",motorLeft, motorRight,left_x,right_x);

    speedL = motor_map(left_x);
    speedR = motor_map(right_x);
    printf("speedL %d speedR %d dirL %d dirR %d aibotgo %d Running(%d)(%d) Safety(%d)(%d) ", 
	            speedL,speedR, dirL, dirR, aibotgo, RunningMode_F, 
				RunningMode_B, SaftyMode_F, SaftyMode_B);
    memset((void*)&canmsg, 0x00, sizeof(struct can_frame));
    canmsg.can_id = MSG_AIBOT_WHEEL;

    canmsg.data[0] = speedR;
    canmsg.data[1] = 0;  
    canmsg.data[2] = dirR;
    canmsg.data[3] = aibotgo; 
    canmsg.data[4] = speedL;
    canmsg.data[5] = 0;  
    canmsg.data[6] = dirL;
    canmsg.data[7] = aibotgo; 	
  //ROS_INFO("[L]speedCmd= %x dir= %x [R]speedCmd= %x dir= %x \n", canmsg.data[0],
      //canmsg.data[2],canmsg.data[4], canmsg.data[6]);  
	  
	printf("canmsg:%d.%d.%d.%d ", canmsg.data[0],canmsg.data[1],canmsg.data[2],canmsg.data[3]);
	printf("  %d.%d.%d.%d\n", canmsg.data[4],canmsg.data[5],canmsg.data[6],canmsg.data[7]);

#if 1
    if (send_port(&canmsg)) {
        //printf("message sent !!\n");
    } else {
        printf("send fail L\n");
    } 
#endif 
    return 0;   
}

int recvCan2Odom(int32_t * LeftEncoder, int32_t *RightEncoder)
{
    //printf("[recvCan2Odom]\n");
     struct can_frame rcvcanmsg;
     memset((void*)&rcvcanmsg, 0x00, sizeof(struct can_frame));
#if 1
     //printf("waiting message..\n");
     int ret = read_port(&rcvcanmsg);
     if(ret <= 0) {
        printf("no message. stopped.\n");
        return -1;
     } 
     else {
	    if (rcvcanmsg.can_id == 0x11) {
	       int32_t left_enc = interprete32((uint8_t*)rcvcanmsg.data);
	       int32_t right_enc = interprete32((uint8_t*)&rcvcanmsg.data[4]);
		   
		   left_enc = -left_enc;
		   left_enc -= CAN_ENC_OFFSET;
		   right_enc -= CAN_ENC_OFFSET;

	       //printf("\n[RECV] left_enc: %d right_enc: %d\n", left_enc, right_enc);
	       //printf("[RECV OFFSET] left_enc: %d right_enc: %d\n", left_enc, right_enc);

	       *LeftEncoder = left_enc;
		   *RightEncoder = right_enc;
       }	
    }  
#else
	       *LeftEncoder = 100;
		   *RightEncoder = 100;
#endif	
	   
    return 0;
}


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  printf("hello ros_rsaem_node package\n");

  set_signal();

  gRun = true;
  
  motorLeft = 1.0;
  motorRight = 1.0;
  printf("[main] motorLeft %f motorRight %f\n",motorLeft, motorRight);

  rsaembot_setup();
  
  /*(Wheels _wheels;
  _wheels.separation = 0.09; // 0.160;
  _wheels.radius = 0.04;  //0.033;*/

  
  rclcpp::init(argc, argv);
  
  rclcpp::spin(std::make_shared<RSaemBot>());

#if 0
  auto mc_node = std::make_shared<MotorCtlSubscriber>();
  
  auto to_node = std::make_shared<TeleopSubscriber>();
   
  auto drive_con =
    std::make_shared<DiffDriveController>(
    _wheels.separation,
    _wheels.radius);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(mc_node);
  executor.add_node(to_node);
  executor.add_node(drive_con);
  executor.spin();       
   //rclcpp::spin(node); 
#endif 
  
#if 1

  pthread_t thread_t2;
  if (pthread_create(&thread_t2, NULL, rsaembot_loop, 0) < 0)
  {
    printf("thread2 create error:");
    exit(0);
  }

  int status2;
  pthread_join(thread_t2, (void **)&status2);
  printf("Thread2 End %d\n", status2);
#endif

  close_rsaembot();  
  rclcpp::shutdown();
  return 0;
}

