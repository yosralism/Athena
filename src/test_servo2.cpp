#include "ros/ros.h"
#include "athena/copter.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "test_servo2");
    ros::NodeHandle nh;
    ros::Rate rate(100);
    
    Copter athena;
    int counter = 1;
    
     
    while(ros::ok){
        
        ros::spinOnce();
         
        athena.left_servo_drop(); 
        athena.right_servo_drop();
	athena.left_servo_drop(); 
        athena.right_servo_drop();
	athena.left_servo_drop(); 
        athena.right_servo_drop();
	athena.left_servo_drop(); 
        athena.right_servo_drop();

	rate.sleep();
    }


    return 0;
}

