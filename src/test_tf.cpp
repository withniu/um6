#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

ros::Subscriber sub;
ros::Publisher pub;	
tf::TransformListener* listener;


//void callback(const ros::TimerEvent&)
void callback(const sensor_msgs::Imu msg) 
{
	tf::StampedTransform transform;

	//Setup Quaternion
	tf::Quaternion quat_in;
	tf::quaternionMsgToTF(msg.orientation, quat_in);
	double roll, pitch, yaw;
	tf::Matrix3x3(quat_in).getRPY(roll, pitch,yaw);
	geometry_msgs::Quaternion quat_out;	
//	ROS_INFO("Before:r,p,y:%f,%f,%f",roll,pitch,yaw);

	try {
		listener->lookupTransform("base_link", "imu_link", ros::Time(0), transform);
		transform.setOrigin(tf::Vector3(0,0,0));
	} catch (tf::TransformException &ex) {
		ROS_DEBUG("Missed a transform");
		return;
	}


	//robot_pose_ekf method
	tf::Transform imu_meas_;
    imu_meas_ = tf::Transform(quat_in, tf::Vector3(0,0,0));
    imu_meas_ = imu_meas_ * transform;
	tf::quaternionTFToMsg(imu_meas_.getRotation(), quat_out);


	//Chad Rockey Method
/*	tf::Quaternion quat_tf;
	tf::quaternionTFToMsg(transform*quat_in, quat_out);*/


	geometry_msgs::PoseStamped pose_out; 
	pose_out.header.stamp = msg.header.stamp;
	pose_out.header.frame_id = "base_link";
	pose_out.pose.orientation = quat_out;
	pub.publish(pose_out);
	
//	ROS_INFO("x,y,z,w:%f,%f,%f,%f",quat_out.x,quat_out.y,quat_out.z,quat_out.w);

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_transformer");
	ros::NodeHandle n;
//	ros::Timer timer = n.createTimer(ros::Duration(1), callback);
	listener = new tf::TransformListener;
	sub = n.subscribe("imu/data", 1, callback);	
	pub = n.advertise<geometry_msgs::PoseStamped>("pose_transformed",1000);
	ros::spin();
}

