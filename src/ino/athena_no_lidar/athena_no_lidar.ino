#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <Servo.h>


Servo servo_left;
Servo servo_right;

ros::NodeHandle nh;

void servo_left_callback(const std_msgs::Int8& data){

  if (data.data == 1){
    servo_left.attach(11);
    servo_left.write(0);

  }
  else if (data.data == 3){
    servo_left.detach();  
  }

}

void servo_right_callback(const std_msgs::Int8& data){

  if (data.data == 2){
    servo_right.attach(9);
    servo_right.write(180);

  }
  else if (data.data == 3){
    servo_right.detach();  
  }

}

ros::Subscriber<std_msgs::Int8> servo_left_sub("left_servo", &servo_left_callback);
ros::Subscriber<std_msgs::Int8> servo_right_sub("right_servo", &servo_right_callback);

void setup()
{
  Serial.begin(57600); // Start serial communications

  nh.initNode();
  nh.subscribe(servo_left_sub);
  nh.subscribe(servo_right_sub);  
}


void loop()
{
    nh.spinOnce();
    delay(10);
}
