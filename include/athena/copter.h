#ifndef COPTER
#define COPTER

#include <string>

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/TwistStamped.h"
#include "mavros_msgs/SetMode.h"
#include "athena/Object.h"

class Copter {
  public:
    Copter();
    ~Copter();

    // Copter movement methods
    void go_center();
    void go_down();
    void go_up();
    void change_flight_mode(std::string mode);
    void left_servo_drop();
    void right_servo_drop();

    // Callback
    void cv_target_callback(const athena::Object& obj_loc);
    void lidar_alt_callback(const std_msgs::Int16& data);
    void timer_callback(const ros::TimerEvent& event);
    void control_effort_x_callback(const std_msgs::Float64& data);
    void control_effort_y_callback(const std_msgs::Float64& data);
    void control_effort_z_callback(const std_msgs::Float64& data);

  private:
    ros::Publisher _cmd_vel_publisher;
    ros::Publisher _state_x_publisher;
    ros::Publisher _state_y_publisher;
    ros::Publisher _state_z_publisher;
    ros::Publisher _setpoint_x_publisher;
    ros::Publisher _setpoint_y_publisher;
    ros::Publisher _setpoint_z_publisher;

    ros::Publisher _left_servo_publisher;
    ros::Publisher _right_servo_publisher;
    
    ros::Subscriber _cv_target_subscriber;
    ros::Subscriber _lidar_alt_subscriber;
    ros::Subscriber _control_effort_x_subscriber;
    ros::Subscriber _control_effort_y_subscriber;
    ros::Subscriber _control_effort_z_subscriber;
    
    ros::ServiceClient _set_mode_client;
    
    ros::Timer _mission_timer;

    // Detected Object to be subscribed from _cv_det_sub
    int _x_det = 0;
    int _y_det = 0;

    // Control Effort to be subscribed from its corresponding controller node
    double _control_effort_x;
    double _control_effort_y;
    double _control_effort_z;

    // Camera center location
    const int _X_CAM = 200;
    const int _Y_CAM = 200;

	
    //which servo should drop the log, 1 for left_servo and 2 for right_servo

    int servo;
    
    // ROS Timer
    double _mission_time = 20.;    // seconds
    bool _mission_timeout = false; // flag when timer finishs counting

    // Altitude data
    int _copter_alt = 0;         // will be filled by arduino data
    int _desired_alt_up = 400;   // for go_up() method
    int _desired_alt_down = 200; // for go_down() method
};

#endif
