#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <Servo.h>


Servo servo_left;
Servo servo_right;

void left_servo_callback(const std_msgs::Int8& data){
  if (data.data == 2){
    servo_left.attach(11);
    servo_left.write(0);
  }
  else if (data.data == 3){
    servo_left.detach();
  }
}

void right_servo_callback(const std_msgs::Int8& data){

  if (data.data == 1){
    servo_right.attach(9);
    servo_right.write(180);

  }
  else if (data.data == 3){
    servo_right.detach();
  }
}

ros::NodeHandle nh;


std_msgs::Int16 altitude;

ros::Publisher pub_lidar("lidar_alt", &altitude); // definition of the ROS publisher
ros::Subscriber<std_msgs::Int8> left_servo_sub("left_servo", &left_servo_callback);
ros::Subscriber<std_msgs::Int8> right_servo_sub("right_servo", &right_servo_callback);


float alpha = 0.75;
const int trigPin = 2;
const int echoPin = 3;

/*Kabel Coklat echo : Pin 3
  Kabel Kuning trigger : Pin 2
  Kabel Merah : +5v
  Kabel Coklat lakban : Ground */
unsigned long pulseWidth;
unsigned long distanceNow;
unsigned long distanceBefore;

void setup()
{
  Serial.begin(57600); // Start serial communications

  pinMode(trigPin, OUTPUT); // Set pin 2 as trigger pin
  pinMode(echoPin, INPUT); // Set pin 3 as monitor pin
  digitalWrite(trigPin, LOW); // Set trigger LOW for continuous read

  nh.initNode();
  nh.advertise(pub_lidar);
  nh.subscribe(left_servo_sub);
  nh.subscribe(right_servo_sub);
  
}



void loop()
{
/*  pulseWidth = pulseIn(echoPin, HIGH); // Count how long the pulse is high in microseconds

  // If we get a reading that isn't zero, let's print it
  if (pulseWidth != 0 )
  {
    distanceNow = (alpha*(pulseWidth/10)) + ((1-alpha)*distanceBefore);   //10usec = 1 cm of distance
    //Serial.print("Distance = ");    //Print the distance
    //Serial.println(distanceNow);
    distanceBefore = distanceNow;
    //pulseWidth = pulseWidth / 10;
    //range_msg.range = pulseWidth; // 10usec = 1 cm of distance
    altitude.data = distanceNow;
    pub_lidar.publish(&altitude);

  }*/
    altitude.data = 0q;
    pub_lidar.publish(&altitude);
    nh.spinOnce();
    delay(1); 


}
