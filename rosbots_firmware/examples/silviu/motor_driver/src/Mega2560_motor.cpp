/**
 * Motor Driver Example
 */

#include <ros.h>
#include "Arduino.h"

#include <std_msgs/Float32.h>

ros::NodeHandle nh;

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

#define M_LEFT_PWM 12
#define M_LEFT_AIN1 10
#define M_LEFT_AIN2 11
#define M_LEFT_PWM 9
#define M_LEFT_AIN1 8
#define M_LEFT_AIN2 7

void turnWheel( const std_msgs::Float32 &wheel_power,
                unsigned int pwm_pin,
                unsigned int in1_pin, unsigned int in2_pin) {
    float factor = max(min(wheel_power.data, 1.0f), -1.0f);
    if( factor >= 0 ) {
        //digitalWrite(fr_pin, LOW);
        digitalWrite(in1_pin, HIGH);
        digitalWrite(in2_pin, LOW);
        analogWrite(pwm_pin, (unsigned int)(255 * factor));
    } else {
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, HIGH);
        analogWrite(pwm_pin, (unsigned int)(255 * (1.0f + factor)));
    }   
}
void rightWheelCb( const std_msgs::Float32 &wheel_power ) {
    //nh.loginfo("Wheel Power - Right");
    //char result[8];
    //dtostrf(wheel_power.data, 6, 2, result); 
    //nh.loginfo(result);
    turnWheel( wheel_power, M_LEFT_PWM, M_LEFT_AIN1, M_LEFT_AIN2 );
}
void leftWheelCb( const std_msgs::Float32 &wheel_power ) {
    //nh.loginfo("Wheel Power - Left");
    turnWheel( wheel_power, M_RIGHT_PWM, M_RIGHT_AIN1, M_RIGHT_AIN2 );
}
ros::Subscriber<std_msgs::Float32> sub_right("wheel_power_right",
                                            &rightWheelCb );
ros::Subscriber<std_msgs::Float32> sub_left("wheel_power_left",
                                           &leftWheelCb );


void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(M_LEFT_PWM, OUTPUT);
  pinMode(M_LEFT_AIN1, OUTPUT);
  pinMode(M_LEFT_AIN2, OUTPUT);
  pinMode(M_RIGHT_PWM, OUTPUT);
  pinMode(M_RIGHT_AIN1, OUTPUT);
  pinMode(M_RIGHT_AIN2, OUTPUT);

  // Init motors to stop
  digitalWrite(M_LEFT_AIN1, LOW);
  digitalWrite(M_LEFT_AIN2, LOW);
  digitalWrite(M_RIGHT_AIN1, LOW);
  digitalWrite(M_RIGHT_AIN2, LOW);
  analogWrite(M_LEFT_PWM, 0);
  analogWrite(M_RIGHT_PWM, 0);

  nh.initNode();
  nh.subscribe(sub_right);
  nh.subscribe(sub_left);
}

void loop()
{
  //nh.loginfo("Log Me");
  nh.spinOnce();

  // wait for a second
  //delay(1000);
}
