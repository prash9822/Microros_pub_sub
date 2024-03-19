#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <string.h>


#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rcl/error_handling.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32_multi_array.h> 
#include <std_srvs/srv/set_bool.h>



#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#endif

#define en 4
byte read=0;

// declaring motor pwm variables (these are used for sending pwm signals to motor driver)
int FL_motor;
int FR_motor;
int BL_motor;
int BR_motor;

// declaring pins for motor driver (Arduino_Due)
#define mPinFL 9
#define mPinFR 10
#define mPinBL 8
#define mPinBR 11
#define dirFL 29
#define dirFR 31
#define dirBL 27
#define dirBR 33

#define pwmPin1 21
#define dir1 22
#define pwmPin2 2
#define dir2 3
#define pwmPin3 23
#define dir3 25
#define pwmPin4 13
#define dir4 3

rcl_publisher_t publisher_imu;
rcl_publisher_t publisher_line;
rcl_publisher_t publisher_luna;

std_msgs__msg__Int32 lsa08; 
std_msgs__msg__Int32MultiArray msg_lsa08;        
sensor_msgs__msg__Imu imu_msg;

rcl_subscription_t subscriber;       
geometry_msgs__msg__Twist sub_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


rcl_timer_t timer_line;
rcl_timer_t timer_imu;
rcl_timer_t timer_luna;

rcl_service_t service;
rcl_wait_set_t wait_set;

std_srvs__srv__SetBool_Response req;
std_srvs__srv__SetBool_Response res;

Adafruit_MPU6050 mpu;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void pinSetup()
{
  pinMode(en,OUTPUT);
  pinMode(mPinFL, OUTPUT);
  pinMode(mPinFR, OUTPUT);
  pinMode(mPinBL, OUTPUT);
  pinMode(mPinBR, OUTPUT);

  pinMode(dirFL, OUTPUT);
  pinMode(dirFR, OUTPUT);
  pinMode(dirBL, OUTPUT);
  pinMode(dirBR, OUTPUT);

  pinMode(dir1, OUTPUT);
  pinMode(pwmPin1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(dir3, OUTPUT);
  pinMode(pwmPin3, OUTPUT);
  pinMode(dir4, OUTPUT);
  pinMode(pwmPin4, OUTPUT);

}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}
void rotate_clockwise(bool &success)
{
  digitalWrite(dir1, HIGH);
  digitalWrite(dir2, HIGH);
  digitalWrite(dir3, HIGH);
  digitalWrite(dir4, HIGH);
  analogWrite(pwmPin1, 255);
  analogWrite(pwmPin2, 255);
  analogWrite(pwmPin3, 255);
  analogWrite(pwmPin4, 255);
  delay(3000);
  analogWrite(pwmPin1, 0);
  analogWrite(pwmPin2, 0);
  analogWrite(pwmPin3, 0);
  analogWrite(pwmPin4, 0);
  success = true;
}

void rotate_anticlockwise(bool &success)
{
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, LOW);
  digitalWrite(dir3, LOW);
  digitalWrite(dir4, LOW);
  analogWrite(pwmPin1, 255);
  analogWrite(pwmPin2, 255);
  analogWrite(pwmPin3, 255);
  analogWrite(pwmPin4, 255);
  delay(3000);
  analogWrite(pwmPin1, 0);
  analogWrite(pwmPin2, 0);
  analogWrite(pwmPin3, 0);
  analogWrite(pwmPin4, 0);
  success = true;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
     
      Serial1.begin(115200);
      digitalWrite(en,LOW);
      while(Serial1.available()<=0);
      read=Serial1.read();
      // Serial.println(read);
      lsa08.data=read;
      digitalWrite(en,HIGH);
      if (timer!= NULL){

       RCSOFTCHECK(rcl_publish(&publisher_line, &lsa08, NULL));
      
  }
}

void timer_callback_imu(rcl_timer_t * timer, int64_t last_call_time) {

    RCLC_UNUSED(last_call_time);

  //   if (!mpu.begin()) {
  //   // Serial.println("Failed to find MPU6050 chip");
  //   while (1) {
  //     delay(10);
  //   }
  // }
     
     mpu.begin();
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    imu_msg.linear_acceleration.x = a.acceleration.x;
    imu_msg.linear_acceleration.y = a.acceleration.y;
    imu_msg.linear_acceleration.z = a.acceleration.z;

    imu_msg.angular_velocity.x = g.gyro.x;
    imu_msg.angular_velocity.y = g.gyro.y;
    imu_msg.angular_velocity.z = g.gyro.z;
    
      if (timer != NULL) {
     
     RCSOFTCHECK(rcl_publish(&publisher_imu, &imu_msg, NULL));

  }
}

void timer_callback_multiarray(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);

   // Clear previous data
   msg_lsa08.data.size = 0;
   msg_lsa08.data.capacity = 0;
   msg_lsa08.data.data = NULL;
    
    // Add new data
    msg_lsa08.data.data = (int32_t*)malloc(sizeof(int32_t) * 3); // Assuming you want to publish 3 values
    if  (msg_lsa08.data.data != NULL) {
     msg_lsa08.data.size = 3;
     msg_lsa08.data.capacity = 3;
     msg_lsa08.data.data[0] = 1; // Example value
     msg_lsa08.data.data[1] = 2; // Example value
     msg_lsa08.data.data[2] = 3; // Example value

  if (timer != NULL) {
      // Publish the message
      RCSOFTCHECK(rcl_publish(&publisher_luna, &msg_lsa08, NULL));
    }
  }
}

void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
   float x1 = msg->linear.x;
   float y1 = msg->linear.y;
   float z1 = msg->angular.z;

  float mapped_leftHatx =  (100.0/2.0)*x1;
  float mapped_leftHaty = (100.0/2.0)* y1;
  float mapped_rightHatz = (50.0/2.0)* z1;

    FL_motor = mapped_leftHatx - mapped_rightHatz + mapped_leftHaty;
    BR_motor = mapped_leftHatx + mapped_rightHatz + mapped_leftHaty;
    FR_motor = mapped_leftHatx + mapped_rightHatz - mapped_leftHaty;
    BL_motor = mapped_leftHatx - mapped_rightHatz - mapped_leftHaty;
    
    // constraining motor variables between -255 to 255
    FL_motor = constrain(FL_motor, -255, 255);
    BR_motor = constrain(BR_motor, -255, 255);
    FR_motor = constrain(FR_motor, -255, 255);
    BL_motor = constrain(BL_motor, -255, 255);

    // assigning direction values
    // HIGH - Backward && LOW - Forward
    if(FL_motor < 0)
    {
      digitalWrite(dirFL,HIGH);
      // Serial.println("HIGH");
    } else {
      digitalWrite(dirFL,LOW);
      // Serial.println("LOW");
    }

    if(BR_motor < 0)
    {
      digitalWrite(dirBR,HIGH);
    } else {
      digitalWrite(dirBR,LOW);
    }

    if(FR_motor < 0)
    {
      digitalWrite(dirFR,HIGH);
      // Serial.println("HIGH");
    } else {
      digitalWrite(dirFR,LOW);
    }

    if(BL_motor < 0)
    {
      digitalWrite(dirBL,HIGH);
      // Serial.println("HIGH");
    } else {
      digitalWrite(dirBL,LOW);
    }

    // printing pwm values
//    Serial.print("FL : ");
//    Serial.print(FL_motor);
//    Serial.print("  FR : ");
//    Serial.println(FR_motor);
//    Serial.print("BL : ");
//    Serial.print(BL_motor);
//    Serial.print("  BR : ");
//    Serial.println(BR_motor);

    FL_motor = abs(FL_motor);
    BR_motor = abs(BR_motor);
    FR_motor = abs(FR_motor);
    BL_motor = abs(BL_motor);
    
    // analogWrite those pwm values and write cases for Direction pins
    analogWrite(mPinFL,FL_motor);
    analogWrite(mPinFR,FR_motor);
    analogWrite(mPinBL,BL_motor);
    analogWrite(mPinBR,BR_motor);

}
void service_callback(const void * req, void * res){
  
  
  std_srvs__srv__SetBool_Request * req_in=(std_srvs__srv__SetBool_Request *) req;
  std_srvs__srv__SetBool_Response * res_in=(std_srvs__srv__SetBool_Response *) res;

  //printf("Service request value: %d + %d.\n", (int) req_in->a, (int) req_in->b);
//  String response_str = "f1";
  bool success = false;

  if(req_in->data == 1)
  {
//    res_in->success = true;
//    res_in->message = response_str;

      rotate_clockwise(success);
      res_in->success = success;
    
  }

  else
  {
//    res_in->success = true;
//    res_in->message = "f0";

      rotate_anticlockwise(success);
      res_in->success = success;
  }

}

void setup() {
   
  pinSetup();
  // Configure serial transport
 
  set_microros_serial_transports(Serial);
  delay(1000);
  
  Serial.begin(115200);
  
 // Try to initialize!

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node 
  RCCHECK(rclc_node_init_default(&node, "r2_vrc", "", &support));

    // create service
   RCCHECK(rclc_service_init_default(
    &service, 
    &node, 
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), 
    "/setbool"));


  // create publisher lsa08 data
  RCCHECK(rclc_publisher_init_default(
    &publisher_line,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "line_lsa"));

 // creating publisher for imu_sensor data

    RCCHECK(rclc_publisher_init_default(
    &publisher_imu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_info_topic"));
  
  //create publisher for lunar sensor

    RCCHECK(rclc_publisher_init_default(
    &publisher_luna,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "luna_data"));

  // subscriber for cmd vel
    RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

    //create timer for lunar sensor

    const unsigned int timer_timeout_luna = 500;
    RCCHECK(rclc_timer_init_default(
    &timer_luna,
    &support,
    RCL_MS_TO_NS(timer_timeout_luna),
    timer_callback_multiarray));

  // create timer for LSA08,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer_line,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

 // create timer for mpu6050,
  const unsigned int timer_timeout_imu = 500;
  RCCHECK(rclc_timer_init_default(
    &timer_imu,
    &support,
    RCL_MS_TO_NS(timer_timeout_imu),
    timer_callback_imu));

  // create executor
  
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_line));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback));

  RCCHECK(rclc_executor_add_timer(&executor, &timer_luna));
  

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

 delay(100);
  
}

void loop() {

  delay(100);

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}