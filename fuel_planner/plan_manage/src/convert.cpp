#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include<trajectory_msgs/MultiDOFJointTrajectory.h>
#include "quadrotor_msgs/PositionCommand.h"
#include<trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
using namespace std;
// gazebo_msgs::LinkStates state;
geometry_msgs::PoseStamped rpy;
ros::Publisher pub;
ros::Subscriber sub;
trajectory_msgs::MultiDOFJointTrajectory cmd;

geometry_msgs::Transform t;
geometry_msgs::Twist vel,acc;

void clbk_fn(const quadrotor_msgs::PositionCommand& msg) {

	tf2::Quaternion q;
	// cmd.joint_names[0] = "";
	cmd.points.clear();
	trajectory_msgs::MultiDOFJointTrajectoryPoint point;
	t.translation.x = msg.position.x;
	t.translation.y = msg.position.y;
	t.translation.z = msg.position.z;
	
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
	cout << "iamhere";
	pub.publish(cmd);
	

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "converter");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 50);
    sub = nh.subscribe("position_cmd", 50, clbk_fn);
    while (ros::ok()) {
        ros::spinOnce();
	// cmd.points.push_back(point);
	// t.translation.x = 0;
	// t.translation.y = 0;
	// t.translation.z = 0;
	
	// vel.linear.x = 1;
	// vel.linear.y = 0;
	// vel.linear.z = 0;
	// vel.angular.z =1;

	// acc.linear.x = 0;
	// acc.linear.y = 0;
	// acc.linear.z = 0;

	
	// // cmd.header.stamp.sec = ros::Time::now().toSec();
	// pub.publish(cmd);
        rate.sleep();
    }
    return 0;
}
