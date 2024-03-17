/****************************---------------------AUTONOMOUS_ROBOT_2--------------------************************************************\

                                           *             *
                                    FL    * * * * * * * * *  FR
                                         *  *           *  *
                                            *           *
                                            *    R2     *
                                            *           *
                                         *  *           *  *
                                    BL    * * * * * * * * *   BR
                                           *             *
                                            
/*****************************************************************************************************************************************/
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#endif

#define en 4
byte read=0;

// declaring motor pwm variables (these are used for sending pwm signals to motor driver)
int FL_motor;
int FR_motor;
int BL_motor;
int BR_motor;

float x ;
float y ;
float z ;

// declaring pins for motor driver (Arduino_Due)
#define mPinFL 9
#define mPinFR 10
#define mPinBL 8
#define mPinBR 11
#define dirFL 29
#define dirFR 31
#define dirBL 27
#define dirBR 33

rcl_publisher_t publisher_imu;
rcl_publisher_t publisher_line;

std_msgs__msg__Int32 lsa08;
sensor_msgs__msg__Imu imu_msg;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist sub_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_timer_t timer_line;
rcl_timer_t timer_imu;

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
}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
     
     digitalWrite(en,LOW);
      while(Serial1.available()<=0);
      read=Serial1.read();
      // Serial.println(read);
      lsa08.data=read;
      digitalWrite(en,HIGH);
      if (timer != NULL){

       RCSOFTCHECK(rcl_publish(&publisher_line, &lsa08, NULL));
      
  }
}
void timer_callback_imu(rcl_timer_t * timer, int64_t last_call_time) {

    RCLC_UNUSED(last_call_time);
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

void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
   x = msg->linear.x;
   y = msg->linear.y;
   z = msg->angular.z;

  float mapped_leftHatx = map(x,0,2,0,100);
  float mapped_leftHaty = map(y,0,2,0,100);
  float mapped_rightHatz = map(z,0,1,0,50);

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

void setup() {
   
  pinSetup();
  // Configure serial transport
  Serial.begin(115200);
  Serial1.begin(115200);
  
  set_microros_serial_transports(Serial);
  delay(2000);

 // Try to initialize!

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  // Serial.println("MPU6050 Found!");

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "r2_vrc", "", &support));

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

  // subscriber
    RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create timer,
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
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_line));
   RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA));

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  delay(100);
  
}

void loop() {

  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}