#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include<trajectory_msgs/MultiDOFJointTrajectory.h>
#include "quadrotor_msgs/PositionCommand.h"
#include<trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
using namespace std;
// gazebo_msgs::LinkStates state;
geometry_msgs::PoseStamped rpy;
ros::Publisher pub, pub_pose;
ros::Subscriber sub;
trajectory_msgs::MultiDOFJointTrajectory cmd;

geometry_msgs::Transform t;
geometry_msgs::Twist vel,acc;


void clbk_fn(const quadrotor_msgs::PositionCommand& msg) {

	tf2::Quaternion q;
	// cmd.joint_names[0] = "";
	q.setRPY(0,0,msg.yaw);
	q.normalize();
	cmd.points.clear();
	trajectory_msgs::MultiDOFJointTrajectoryPoint point;
	t.translation.x = msg.position.x;
	t.translation.y = msg.position.y;
	t.translation.z = msg.position.z;

	t.rotation.x = q.x();
	t.rotation.y = q.y();
	t.rotation.z = q.z();
	t.rotation.w = q.w();
	
	vel.linear.x = msg.velocity.x;
	vel.linear.y = msg.velocity.y;
	vel.linear.z = msg.velocity.z;
	vel.angular.z = msg.yaw_dot;

	acc.linear.x = msg.acceleration.x;
	acc.linear.y = msg.acceleration.y;
	acc.linear.z = msg.acceleration.z;

	point.transforms.push_back(t);
	point.velocities.push_back(vel);
	point.accelerations.push_back(acc);
	cmd.points.push_back(point);

	cmd.header.stamp.sec = ros::Time::now().toSec();
	// if(std::abs(msg.position.z-0)>0.1)
	pub.publish(cmd);
	

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "converter");
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10);
    pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 50);
    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/firefly/sensor_pose",10);
    sub = nh.subscribe("/planning/pos_cmd", 50, clbk_fn);
    while (ros::ok()) {
        ros::spinOnce();
	geometry_msgs::TransformStamped transformStamped;
	geometry_msgs::PoseStamped msg_pub;

      	try{
        	transformStamped = tfBuffer.lookupTransform("world", "firefly/depth_camera",ros::Time(0));
      	}
      	catch (tf2::TransformException &ex) {
        	ROS_WARN("%s",ex.what());
		rate.sleep();
        	continue;
      	}
	
	msg_pub.pose.position.x = transformStamped.transform.translation.x;
	msg_pub.pose.position.y = transformStamped.transform.translation.y;
	msg_pub.pose.position.z = transformStamped.transform.translation.z;
	msg_pub.pose.orientation.x = transformStamped.transform.rotation.x;
	msg_pub.pose.orientation.y = transformStamped.transform.rotation.x;
	msg_pub.pose.orientation.z = transformStamped.transform.rotation.x;
	msg_pub.pose.orientation.w = transformStamped.transform.rotation.x;

	msg_pub.header.frame_id = transformStamped.header.frame_id;
	msg_pub.header.seq = transformStamped.header.seq;
	msg_pub.header.stamp = transformStamped.header.stamp;

	pub_pose.publish(msg_pub);
	ROS_INFO("Transform from sensor frame to world frame is published");

        rate.sleep();
    }
    return 0;
}
