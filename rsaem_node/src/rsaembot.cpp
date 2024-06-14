// Copyright 2024 JetsonAI CO., LTD.
//
// Author: Kate Kim

#include "rsaem_node/rsaembot.hpp"
#include "rsaem_node/cansocket.hpp"
#include "rsaem_node/rsaem_msgctl/msg/msg_ctl.hpp"

#include <memory>
#include <string>

using jetsonai::rsaem::RSaemBot;
using namespace std::chrono_literals;

class MotoCtlSubscriber : public rclcpp::Node
{
public:
  MotoCtlSubscriber()
  : Node("Helloworld_subscriber")
  {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
//    motorctl_subscriber_ = this->create_subscription<std_msgs::msg::String>(
    motorctl_subscriber_ = this->create_subscription<rsaem_msgctl::msg::MsgCtl>(
      "MotorCtl",
      qos_profile,
      std::bind(&MotoCtlSubscriber::subscribe_topic_message, this, _1));
      count = 0;
  }
  int count;

private:
  //void subscribe_topic_message(const std_msgs::msg::String::SharedPtr msg) const
  void subscribe_topic_message(const rsaem_msgctl::msg::MsgCtl::SharedPtr msg) 
  { 
    RCLCPP_INFO(this->get_logger(), "recieve speedCmd:%0.2f headingCmd:%0.2f dir_cmd:%d stop_cmd:%d", 
         msg->speed_cmd, msg->heading_cmd, msg->dir_cmd , msg->stop_cmd );
 
 motor_contol(msg->speed_cmd, msg->heading_cmd, msg->dir_cmd, msg->stop_cmd);

  }
  rclcpp::Subscription<rsaem_msgctl::msg::MsgCtl>::SharedPtr motorctl_subscriber_;

};


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
    //motor_contol(0,0,0,1);
    //close_port();
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

int RSaemBot::sendCanTx2(float speed, int dir, float angle, int aibotgo)
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


RSaemBot::RSaemBot(const std::string & can_port)
: Node("rsaem_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  RCLCPP_INFO(get_logger(), "Init TurtleBot3 Node Main");
  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  //init_dynamixel_sdk_wrapper(usb_port);
  init_can(can_port)
  check_device_status();

  //we don't need this funtion 20240530// add_motors();
  add_wheels();
  //we don't have sensors for this funtion 20240530// add_sensors();
  add_devices();

  run();
}



void RSaemBot::run()
{
  RCLCPP_INFO(this->get_logger(), "Run!");

  publish_timer(std::chrono::milliseconds(50));
  heartbeat_timer(std::chrono::milliseconds(100));

  parameter_event_callback();
  cmd_vel_callback();
}



void RSaemBot::parameter_event_callback()
{
  priv_parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
  while (!priv_parameters_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }

    RCLCPP_WARN(this->get_logger(), "service not available, waiting again...");
  }


void RSaemBot::cmd_vel_callback()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    qos,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
    {
      std::string sdk_msg;

      union Data {
        int32_t dword[6];
        uint8_t byte[4 * 6];
      } data;

      data.dword[0] = static_cast<int32_t>(msg->linear.x * 100);
      data.dword[1] = 0;
      data.dword[2] = 0;
      data.dword[3] = 0;
      data.dword[4] = 0;
      data.dword[5] = static_cast<int32_t>(msg->angular.z * 100);

      uint16_t start_addr = extern_control_table.cmd_velocity_linear_x.addr;
      uint16_t addr_length =
      (extern_control_table.cmd_velocity_angular_z.addr -
      extern_control_table.cmd_velocity_linear_x.addr) +
      extern_control_table.cmd_velocity_angular_z.length;

      uint8_t * p_data = &data.byte[0];

      //dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

      RCLCPP_DEBUG(
        this->get_logger(),
        "lin_vel: %f ang_vel: %f msg : %s", msg->linear.x, msg->angular.z, sdk_msg.c_str());
    }
  );
}
