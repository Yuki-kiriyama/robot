/*//////////////////////////////////////////
micro-ROSでDynamxelを動かす

ターミナル１
>>ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
ターミナル２
>>ros2 topic list ←/dx_commandと/dx_statusがあればOK！
>>ros2 topic pub --once /dx_command std_msgs/Int16 "data: 1"
か
>>ros2 topic pub --once /dx_command std_msgs/Int16 "data: 2"
*/////////////////////////////////////////

//micro-ROSのライブラリ
#include <micro_ros_arduino.h>

//Dynamxelのライブラリ
#include <dxlib.h>
#include <dxmemmap.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>

#include "avr_uno_softserial.h"

const

const DXLIB::TDXHost_ConfParam param {
  us_init, us_deinit, us_setbaudrate, us_rxpurge, us_putc, us_puts, us_gets, us_flush
};

DXLIB dx (&param);

typedef struct {
  int16_t minpos, maxpos;
} Tmaxminpos;

Tmaxminpos maxminpos;
int16_t pos_0 = 0;
int16_t pos_45 = 500;
int16_t pos = 0;

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t publisher;
static rcl_subscription_t subscriber;
static rclc_executor_t executor;
static std_msgs__msg__Int16 st_msg; //Dynamxelの状態を知る用
static std_msgs__msg__Int16 dx_msg; //Dynamxelにコマンド送る用

#define TARGET_ID 1
#define BAUDRATE  9600

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED0, !digitalRead(LED0));
    delay(100);
  }
}

//callback
void openclose_callback(const void * msgin) {  
  const std_msgs__msg__Int16 * dx_msg = (const std_msgs__msg__Int16 *)msgin;  

  if (dx_msg->data == 1) {
    dx.WriteWordData(TARGET_ID, ADDRESS_GOAL_POSITION, pos_45, NULL);  // 45°に回転
  } else if (dx_msg->data == 2) {
    dx.WriteWordData(TARGET_ID, ADDRESS_GOAL_POSITION, pos_0, NULL);   // 0°に戻す
  }
}

void setup() {
  Serial.begin (115200);
  dx.begin (BAUDRATE);

  if (dx.ReadBlockData (TARGET_ID, ADDRESS_CW_ANGLE_LIMIT, (uint8_t *)&maxminpos, sizeof (Tmaxminpos), NULL)) {
    if (dx.ReadWordData (TARGET_ID, ADDRESS_PRESENT_POSITION, (uint16_t *)&pos, NULL)) {
      Serial.print ("max pos=");
      Serial.print (maxminpos.minpos);
      Serial.print (" min pos=");
      Serial.print (maxminpos.maxpos);
      Serial.print (" presens pos=");
      Serial.println (pos);
    } else Serial.println ("Failed to ReadWordData");
  } else Serial.println ("Failed to ReadBlockData");
  
  set_microros_transports();
  delay(2000);
  
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  rclc_node_init_default(&node, "dynamixel_node", "", &support);

  // publisher作成
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "dx_status");
 
  // subscriber作成
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "dx_command");

  // create executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);  //引数３つ目はsubscriberの数
  rclc_executor_add_subscription(&executor, &subscriber, &dx_msg, &openclose_callback, ON_NEW_DATA);

 
  digitalWrite(LED0, HIGH);  
}

void loop() {
  delay(100);
  
  st_msg.data = maxminpos.maxpos;
  
  // トピックにパブリッシュ
  RCCHECK(rcl_publish(&publisher, &st_msg, NULL));
  
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
