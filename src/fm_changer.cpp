#include "ros/ros.h"

#include "std_msgs/Int8.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/RCIn.h"


/* ===================================================================
    
    Channel 7 as GUIDED-mode triggerer
       CH7
    =========
      ===== 2000 (CH.7 0N)
      ===== 1900 (UPPER THRESHOLD) : SOMETHING BELOW THIS MEANS "OFF"
       |||
       ||| BANDWIDTH
       |||
      ===== 1000 (BOTTOM THRESHOLD) : SOMETHING ABOVE THIS MEANS "ON"
      =====  900 (CH.7 OFF)
    =========
   =================================================================== */


int OFFSET     = 300;
int RC_IN_CH7;
int RC_CH7_OFF = 900 + OFFSET;
int RC_CH7_ON  = 2000 - OFFSET;

void rc_in_callback (const mavros_msgs::RCIn& rc_data);

int main (int argc, char **argv) {
    ros::init(argc, argv, "fm_changer_test");
    ros::NodeHandle nh;

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    
    ros::Publisher cv_flag_publisher = nh.advertise<std_msgs::Int8>("cv_flag", 1);
    ros::Publisher mission_type_publisher = nh.advertise<std_msgs::Int8>("mission_type", 1);

    ros::Subscriber rc_in_subscriber = nh.subscribe("/mavros/rc/in", 1, rc_in_callback);

    ros::Rate rate(30);     // 30 Hz

    // Initial conditions needs to be fulfilled (ch7 should ON)
    ROS_INFO("fm_changer is waiting for Ch-7");
    while( !(ros::ok() && RC_IN_CH7 > RC_CH7_OFF) ){
        ros::spinOnce();
        rate.sleep();
    }

    /*
    CHANNEL 7 IS TRIGERRED -> CHANGE TO AUTO TO FOLLOW WAYPOINT
    */
    mavros_msgs::SetMode flight_mode;
    flight_mode.request.base_mode = 0;
    /*
    flight_mode.request.custom_mode = "AUTO";
    if (set_mode_client.call(flight_mode))
        ROS_INFO("Flight mode changed to AUTO");
    else
        ROS_INFO("WARNING : Failed to change flight mode to AUTO");
    */

    while(ros::ok()){
        ros::spinOnce();
        /*
        CHANNEL 7 IS UN-TRIGGERED AND IN GUIDED/AUTO MODE -> CHANGE TO LAND
        */
        if (RC_IN_CH7 < RC_CH7_ON) {
            flight_mode.request.custom_mode = "LAND";

            if (set_mode_client.call(flight_mode))
                ROS_INFO("Flight mode changed to LAND. RC is taken over by pilot.");
            else
                ROS_INFO("WARNING : Failed to change flight mode to LAND");
            
            // SHUT DOWN vision.cpp AND mission_control.cpp
            std_msgs::Int8 cv_flag;
            cv_flag.data = -1;
            cv_flag_publisher.publish(cv_flag);
            
            std_msgs::Int8 mission;
            mission.data = -1;
            mission_type_publisher.publish(mission);

            break;
        }

        rate.sleep();
    }

    ROS_INFO("fm_changer_test is shutting down!");
    
    return 0;
}

void rc_in_callback (const mavros_msgs::RCIn& rc_data){
    RC_IN_CH7 = rc_data.channels[6];
}
