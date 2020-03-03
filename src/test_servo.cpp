#include "ros/ros.h"
#include "athena/copter.h"




int main (int argc, char **argv) {
    ros::init(argc, argv, "test_servo");

    Copter athena;
    int counter = 1;

    ros::Rate rate(20); 

    while(ros::ok()){
        
       	athena.left_servo_drop();
     	athena.right_servo_drop();
       
        

        rate.sleep();
	ros::spinOnce();
    } // end of while(ros::ok())
    
    ROS_INFO("test_servo is shutting down!");
    
    return 0;
}


