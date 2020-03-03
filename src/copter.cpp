#include "athena/copter.h"

Copter::Copter(){
    ros::NodeHandle _nh;
    ros::NodeHandle _nh_priv("~");

    _cmd_vel_publisher = _nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    
    _state_x_publisher = _nh.advertise<std_msgs::Float64>("/pid_x/state", 10);
    _state_y_publisher = _nh.advertise<std_msgs::Float64>("/pid_y/state", 10);
    _state_z_publisher = _nh.advertise<std_msgs::Float64>("/pid_z/state", 10);

    _setpoint_x_publisher = _nh.advertise<std_msgs::Float64>("/pid_x/setpoint", 10);
    _setpoint_y_publisher = _nh.advertise<std_msgs::Float64>("/pid_y/setpoint", 10);
    _setpoint_z_publisher = _nh.advertise<std_msgs::Float64>("/pid_z/setpoint", 10);
    
    _cv_target_subscriber   = _nh.subscribe("cv_target", 10, &Copter::cv_target_callback, this);
    _lidar_alt_subscriber   = _nh.subscribe("lidar_alt", 10, &Copter::lidar_alt_callback, this);

    _left_servo_publisher = _nh.advertise<std_msgs::Int8>("left_servo", 10);
    _right_servo_publisher = _nh.advertise<std_msgs::Int8>("right_servo", 10);

    _control_effort_x_subscriber = _nh.subscribe("/pid_x/control_effort", 10, &Copter::control_effort_x_callback, this);
    _control_effort_y_subscriber = _nh.subscribe("/pid_y/control_effort", 10, &Copter::control_effort_y_callback, this);
    _control_effort_z_subscriber = _nh.subscribe("/pid_z/control_effort", 10, &Copter::control_effort_z_callback, this);

    _set_mode_client        = _nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    _mission_timer          = _nh.createTimer(ros::Duration(_mission_time), &Copter::timer_callback, this);
    _mission_timer.stop();

    _nh_priv.param<double>("mission_time", _mission_time, 20.);
    _nh_priv.param<int>("desired_alt_down", _desired_alt_down, 200);
    _nh_priv.param<int>("desired_alt_up", _desired_alt_up, 400);
}

Copter::~Copter(){
    ROS_INFO("COPTER IS ERASED!");
}


/* ===========
    CALLBACKS
   =========== */
/*
    GAK ADA OBJECT krti18::Shape
*/
void Copter::cv_target_callback(const athena::Object& obj_loc){
    _x_det = obj_loc.x_obj;
    _y_det = obj_loc.y_obj;
}

void Copter::lidar_alt_callback(const std_msgs::Int16& data){
    _copter_alt = data.data;
}

void Copter::timer_callback(const ros::TimerEvent& event){
    _mission_timeout = true;
}

void Copter::control_effort_x_callback(const std_msgs::Float64& data){
    _control_effort_x = data.data;
}

void Copter::control_effort_y_callback(const std_msgs::Float64& data){
    _control_effort_y = data.data;
}

void Copter::control_effort_z_callback(const std_msgs::Float64& data){
    _control_effort_z = data.data;
}


/* =========================
    COPTER MOVEMENT METHODS
   ========================= */
void Copter::go_center(){
    ros::Rate temp_rate(30); // 30Hz

    _mission_timer.setPeriod(ros::Duration(_mission_time));
    _mission_timer.start();

    while(ros::ok() &&
          !_mission_timeout){
        std_msgs::Float64 state_x, state_y;
        std_msgs::Float64 setpoint_x, setpoint_y;

        state_x.data = _X_CAM;
        state_y.data = _Y_CAM;

        setpoint_x.data = _x_det;
        setpoint_y.data = _y_det;

        _state_x_publisher.publish(state_x);
        _state_y_publisher.publish(state_y);

        _setpoint_x_publisher.publish(setpoint_x);
        _setpoint_y_publisher.publish(setpoint_y);

        ros::spinOnce();

        // Publish output value from controller node
        geometry_msgs::TwistStamped vel;
        vel.header.stamp = ros::Time::now();
        vel.header.frame_id = "1";
        vel.twist.linear.x =-_control_effort_y;
        vel.twist.linear.y =- _control_effort_x;
        _cmd_vel_publisher.publish(vel);
        
        temp_rate.sleep();
    }

    if(_mission_timeout) ROS_INFO("Mission Timeout!");

    // Reset timer
    _mission_timer.stop();
    _mission_timeout = false;
}

void Copter::go_down(){
    ros::Rate temp_rate(30); // 30Hz

    _mission_timer.setPeriod(ros::Duration(_mission_time));
    _mission_timer.start();

    while(ros::ok() &&
          !_mission_timeout){
        std_msgs::Float64 state_z;
        std_msgs::Float64 setpoint_z;

        state_z.data = _copter_alt;
        setpoint_z.data = _desired_alt_down;

        _state_z_publisher.publish(state_z);
        _setpoint_z_publisher.publish(setpoint_z);

        ros::spinOnce();

        // Publish output value from controller node
        geometry_msgs::TwistStamped vel;
        vel.header.stamp = ros::Time::now();
        vel.header.frame_id = "1";
        vel.twist.linear.z = _control_effort_z;
        _cmd_vel_publisher.publish(vel);
        
        temp_rate.sleep();
    }

    if(_mission_timeout) ROS_INFO("Mission Timeout!");

    // Reset timer
    _mission_timer.stop();
    _mission_timeout = false;
}

void Copter::go_up(){
    ros::Rate temp_rate(30); // 30Hz

    _mission_timer.setPeriod(ros::Duration(_mission_time));
    _mission_timer.start();

    while(ros::ok() &&
          !_mission_timeout){
        std_msgs::Float64 state_z;
        std_msgs::Float64 setpoint_z;

        state_z.data = _copter_alt;
        setpoint_z.data = _desired_alt_up;

        _state_z_publisher.publish(state_z);
        _setpoint_z_publisher.publish(setpoint_z);

        ros::spinOnce();

        // Publish output value from controller node
        geometry_msgs::TwistStamped vel;
        vel.header.stamp = ros::Time::now();
        vel.header.frame_id = "1";
        vel.twist.linear.z = _control_effort_z;
        _cmd_vel_publisher.publish(vel);
        
        temp_rate.sleep();
    }

    if(_mission_timeout) ROS_INFO("Mission Timeout!");

    // Reset timer
    _mission_timer.stop();
    _mission_timeout = false;
}

void Copter::change_flight_mode(std::string mode){
    mavros_msgs::SetMode flight_mode;
    flight_mode.request.base_mode = 0;
    flight_mode.request.custom_mode = mode;
    
    if(_set_mode_client.call(flight_mode)) ROS_INFO("Flight mode changed to %s", mode.c_str());
    else ROS_INFO("WARNING : Failed to change flight mode to %s", mode.c_str());
}

void Copter::left_servo_drop(){
   ROS_INFO("Left servo drop the log");
   servo = 2;
   std_msgs::Int8 mission;
   mission.data = servo;
   _left_servo_publisher.publish(mission);
   usleep(1000000);
   servo = 3;
   mission.data = servo;
   _left_servo_publisher.publish(mission);
}

void Copter::right_servo_drop(){
   ROS_INFO("Right servo drop the log");
   servo = 1;
   std_msgs::Int8 mission;
   mission.data = servo;
   _right_servo_publisher.publish(mission);
   usleep(8500000);
   servo = 3;
   mission.data = servo;
   _right_servo_publisher.publish(mission);
}

