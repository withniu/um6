#include "ros/ros.h"
#include "tf/tf.h"




int main(int argc, char **argv)
{
	ros::init(argc, argv, "um6_compass");
	ros::NodeHandle n;
	tf::Quaternion imu_reading = tf::createQuaternionFromRPY(0.4,0.3,1.2);

	double compass_heading = 1.5705;

	tf::Quaternion compass_yaw = tf::createQuaternionFromRPY(0.0,0.0,compass_heading);
	tf::Quaternion diff_yaw = tf::createQuaternionFromRPY(0.0,0.0, compass_heading - tf::getYaw(imu_reading));

	ROS_INFO("IMU Reading:%f,%f,%f,%f",imu_reading.x(),imu_reading.y(),imu_reading.z(),imu_reading.w());
	ROS_INFO("Compass Yaw:%f,%f,%f,%f",compass_yaw.x(),compass_yaw.y(),compass_yaw.z(),compass_yaw.w());
	ROS_INFO("DIFF Yaw:%f,%f,%f,%f",diff_yaw.x(),diff_yaw.y(),diff_yaw.z(),diff_yaw.w());

	double o_roll,o_pitch,o_yaw;
	double n_roll,n_pitch,n_yaw;


	tf::Matrix3x3(imu_reading).getRPY(o_roll, o_pitch, o_yaw);  
	ROS_INFO("Original RPY:%f,%f,%f",o_roll, o_pitch, o_yaw);

	tf::Quaternion new_quaternion = diff_yaw * imu_reading;


	tf::Matrix3x3(new_quaternion).getRPY(n_roll, n_pitch, n_yaw);  
	ROS_INFO("New RPY:%f,%f,%f",n_roll, n_pitch, n_yaw);

		

	ros::spin();

	return 0;
}


