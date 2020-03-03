#include "ros/ros.h"
#include "mavros_msgs/RCIn.h"
#include "athena/copter.h"
#include "std_msgs/Int8.h"

// Mission type as to what movement to execute
int mission_type;
void mission_type_callback (const std_msgs::Int8& data);

// Channel 7 as GUIDED mode trigger (see fm_changer.cpp for details)
int OFFSET     = 300;
int RC_IN_CH7;
int RC_CH7_OFF = 900 + OFFSET;
int RC_CH7_ON  = 2000 - OFFSET;

void rc_in_callback (const mavros_msgs::RCIn& data);

// Height reference in cm
float height_desired   = 145.;
float height_up        = 400.;

int main (int argc, char **argv) {
    ros::init(argc, argv, "mission_guided");
    ros::NodeHandle nh;

    Copter athena;

    ros::Subscriber mission_type_subscriber = nh.subscribe("mission_type", 1, mission_type_callback);
    ros::Subscriber rc_in_subscriber = nh.subscribe("/mavros/rc/in", 1, rc_in_callback);

    ros::Rate rate(20);     // 20 Hz

    ROS_INFO("mission_guided is waiting for Ch-7");
    while( !(ros::ok() && RC_IN_CH7 > RC_CH7_OFF)){
       ros::spinOnce();
       rate.sleep();
    }
    ROS_INFO("Starting mission_guided!");

    while(ros::ok()){
        ros::spinOnce();
        /*
        -1 ==> BREAK
         1 ==> CENTER
         2 ==> DOWN
        */
        if (mission_type == -1) break;
        
        // GO CENTER
        else if(mission_type == 1) {
            athena.change_flight_mode(std::string("GUIDED"));
            athena.go_center();
	    athena.change_flight_mode(std::string("LOITER"));
            usleep(2000000);
            athena.left_servo_drop();
	    // usleep(2000000);
            //athena.change_flight_mode(std::string("GUIDED"));
            //athena.go_center();
            //athena.change_flight_mode(std::string("LOITER"));
            //usleep(2000000);
            athena.change_flight_mode(std::string("AUTO"));

            mission_type = 0;
        }
        
        // GO DOWN
        else if(mission_type == 2) {
            athena.change_flight_mode(std::string("GUIDED"));
            athena.go_center();
	    athena.change_flight_mode(std::string("LOITER"));
            usleep(2000000);
            athena.right_servo_drop();
	    // usleep(2000000);
            //athena.change_flight_mode(std::string("GUIDED"));
            //athena.go_center();
            //athena.change_flight_mode(std::string("LOITER"));
            //usleep(2000000);
            athena.change_flight_mode(std::string("AUTO"));

            mission_type = 0;
        }

        rate.sleep();

    } // end of while(ros::ok())
    
    ROS_INFO("mission_control_test is shutting down!");
    
    return 0;
}

void mission_type_callback (const std_msgs::Int8& data) {
    mission_type = data.data;
}

void rc_in_callback (const mavros_msgs::RCIn& data) {
    RC_IN_CH7 = data.channels[6];
}   
