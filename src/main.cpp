#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "mavros_msgs/WaypointReached.h"

// Indicating Reached Waypoint and Mission
int waypoint_reached = 0;
void mission_reached_callback(const mavros_msgs::WaypointReached& data);

int main(int argc, char **argv){
    ros::init(argc, argv, "waypoint_reached");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    
    ros::Publisher  mission_type_publisher = nh.advertise<std_msgs::Int8>("mission_type", 1);
    ros::Subscriber mission_reached_subscriber = nh.subscribe("/mavros/mission/reached", 1, mission_reached_callback);

    int waypoint_1 = 2;
    int waypoint_2 = 3;
    int waypoint_3 = 4;
    int waypoint_4 = 5;
    int waypoint_5 = 6;
    int waypoint_6 = 7;
    int waypoint_7 = 8;

    nh_priv.param<int>("waypoint_1", waypoint_1, 2);
    nh_priv.param<int>("waypoint_2", waypoint_2, 3);
    nh_priv.param<int>("waypoint_3", waypoint_3, 4);
    nh_priv.param<int>("waypoint_4", waypoint_4, 5);
    nh_priv.param<int>("waypoint_4", waypoint_5, 6);
    nh_priv.param<int>("waypoint_4", waypoint_6, 7);
    nh_priv.param<int>("waypoint_4", waypoint_7, 8);

    bool mission_1_reached = false;
    bool mission_2_reached = false;
    bool mission_3_reached = false;
    bool mission_4_reached = false;
    bool mission_5_reached = false;
    bool mission_6_reached = false;
    bool mission_7_reached = false;

    ros::Rate rate(30);
    ROS_INFO("Starting waypoint_reached!");
    
    int servo_stop = 3;
    int type;
    
    while(ros::ok()){
        ros::spinOnce();

	std_msgs::Int8 mission;
	mission.data = servo_stop;
	mission_type_publisher.publish(mission);
        
        if(waypoint_reached == waypoint_1 && !mission_1_reached){
            std_msgs::Int8 mission;
	    type = 1;
            mission.data = type;
            mission_type_publisher.publish(mission);
            usleep(1000000);

            mission_1_reached = true;
        }
        
        if(waypoint_reached == waypoint_2 && !mission_2_reached){
            std_msgs::Int8 mission;
            type = 2;
            mission.data = type;
            mission_type_publisher.publish(mission);
            usleep(1000000);

            mission_2_reached = true;
        }
        
        if(waypoint_reached == waypoint_3 && !mission_3_reached){
            std_msgs::Int8 mission;
            type = 1;
            mission.data = type;
            mission_type_publisher.publish(mission);
            usleep(1000000);
	
            mission_3_reached = true;
        }
        
        if(waypoint_reached == waypoint_4 && !mission_4_reached){
            std_msgs::Int8 mission;
            type = 2;
            mission.data = type;
            mission_type_publisher.publish(mission);
            usleep(1000000);

            mission_4_reached = true;
        }
	
	if(waypoint_reached == waypoint_5 && !mission_5_reached){
            std_msgs::Int8 mission;
            type = 1;
            mission.data = type;
            mission_type_publisher.publish(mission);
            usleep(1000000);

            mission_5_reached = true;
        }

	if(waypoint_reached == waypoint_6 && !mission_6_reached){
            std_msgs::Int8 mission;
            type = 2;
            mission.data = type;
            mission_type_publisher.publish(mission);
            usleep(1000000);

            mission_6_reached = true;
        }

	if(waypoint_reached == waypoint_7 && !mission_7_reached){
            std_msgs::Int8 mission;
            type = 1;
            mission.data = type;
            mission_type_publisher.publish(mission);
            usleep(1000000);

            mission_7_reached = true;
        }

        if(mission_1_reached &&
           mission_2_reached &&
           mission_3_reached &&
           mission_4_reached &&
	   mission_5_reached &&
	   mission_6_reached &&
           mission_7_reached) break;

        rate.sleep();
    }

    ROS_INFO("waypoint_reached is shutting down!");
    ROS_INFO("Check the WaypointReached.wp_seq when all waypoint is done");

    return 0;
}

void mission_reached_callback(const mavros_msgs::WaypointReached& data){
    waypoint_reached = data.wp_seq;
}
