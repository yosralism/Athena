#include "ros/ros.h"
#include "std_msgs/Int8.h"



/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher  mission_type_publisher = n.advertise<std_msgs::Int8>("mission_type", 1);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 1;
  while (ros::ok())
  {
    
	if (count == 1){
	std_msgs::Int8 msg;
	msg.data = count;
	mission_type_publisher.publish(msg);
	usleep(1000000);
	count = 3;
	msg.data = count;
	mission_type_publisher.publish(msg);
	usleep(2000000);
	count = 2;
	}

	if (count == 2){
	std_msgs::Int8 msg;
	msg.data = count;
	mission_type_publisher.publish(msg);
	usleep(1000000);
	count = 3;
	msg.data = count;
	mission_type_publisher.publish(msg);
	usleep(2000000);
	count = 1;
	}



    ros::spinOnce();

    loop_rate.sleep();
  
  }


  return 0;
}
