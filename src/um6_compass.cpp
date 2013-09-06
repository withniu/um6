#include "ros/ros.h"
#include "tf/tf.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"


class UM6Compass {

public:
	UM6Compass();
	~UM6Compass();

private:
	ros::NodeHandle n_;
	ros::Subscriber imu_sub_;
	ros::Subscriber mag_sub_;
	ros::Publisher imu_pub_;
	
	void imuCallback(const sensor_msgs::ImuConstPtr& data);
	void magCallback(const geometry_msgs::Vector3ConstPtr& data);
	void repackageImuPublish();
	

	//Heading Filter functions
	void initFilter(double heading_meas); //initialize heading fiter


	bool first_mag_reading; //signifies receiving the first magnetometer message
	bool filter_initialized; //after receiving the first measurement, make sure the filter is initialized
	bool gyro_update_complete; //sigfnifies that a gyro update (motion model update) has gone through

	double mag_zero_x, mag_zero_y, mag_zero_z;

	sensor_msgs::Imu curr_imu_reading;

	//Heading Filter variables

	//State and Variance
	double curr_heading;
	double curr_heading_variance;

	//Motion Update Variables
	double heading_prediction;
	double heading_variance_prediction;
	double heading_prediction_variance;
	double last_motion_update_time;

	//Measurement Update Variables
	double yaw_meas_variance;

};


void UM6Compass::imuCallback(const sensor_msgs::ImuConstPtr& data) { 

	//Transform Data and get the yaw direction

	double yaw_gyro_reading =  data->angular_velocity.z;

	//Run Motion Update
	if (filter_initialized) {
		double dt = ros::Time::now().toSec() - last_motion_update_time;
		heading_prediction = curr_heading + yaw_gyro_reading * dt;	//xp = A*x + B*u
		heading_variance_prediction = curr_heading_variance + heading_prediction_variance; //Sp = A*S*A' + R
		gyro_update_complete = true;
	}
	
	curr_imu_reading = (*data);

}


void UM6Compass::magCallback(const geometry_msgs::Vector3ConstPtr& data) {

	//Compensate for hard iron
	double mag_x = data->x - mag_zero_x;
	double mag_y = data->y - mag_zero_y;
	double mag_z = data->z - mag_zero_z;

	//TODO:Transform Data
	geometry_msgs::Vector3 imu_mag = *data; 

	//Retrieve magnetometer heading 
	double heading_meas = atan2(imu_mag.y, imu_mag.x);


	//If this is the first magnetometer reading, initialize filter
	if (!first_mag_reading) {
		//Initialize filter
		initFilter(heading_meas);		
		return;
	}

	//If gyro update (motion update) is complete, run measurement update and publish imu data
	if (gyro_update_complete) {
		
		double kalman_gain = heading_variance_prediction + (1/(heading_variance_prediction + yaw_meas_variance)); //K = Sp*C'*inv(C*Sp*C' + Q)
		curr_heading = 	heading_prediction + kalman_gain*(heading_meas - heading_prediction); //mu = mup + K*(y-C*mup)
		curr_heading_variance = (1-kalman_gain)*heading_variance_prediction;// S = (1-K*C)*Sp
		repackageImuPublish();
		gyro_update_complete = false;
	}

}

void UM6Compass::repackageImuPublish()
{	
	tf::Quaternion imu_reading;
	tf::quaternionMsgToTF(curr_imu_reading.orientation, imu_reading);
	

	double compass_heading = curr_heading;
	tf::Quaternion compass_yaw = tf::createQuaternionFromRPY(0.0,0.0,compass_heading);
	tf::Quaternion diff_yaw = tf::createQuaternionFromRPY(0.0,0.0, compass_heading - tf::getYaw(imu_reading));

	//ROS_INFO("IMU Reading:%f,%f,%f,%f",imu_reading.x(),imu_reading.y(),imu_reading.z(),imu_reading.w());
	//ROS_INFO("Compass Yaw:%f,%f,%f,%f",compass_yaw.x(),compass_yaw.y(),compass_yaw.z(),compass_yaw.w());
	//ROS_INFO("DIFF Yaw:%f,%f,%f,%f",diff_yaw.x(),diff_yaw.y(),diff_yaw.z(),diff_yaw.w());

	//double o_roll,o_pitch,o_yaw;
	//double n_roll,n_pitch,n_yaw;

	//tf::Matrix3x3(imu_reading).getRPY(o_roll, o_pitch, o_yaw);  
	//ROS_INFO("Original RPY:%f,%f,%f",o_roll, o_pitch, o_yaw);

	tf::Quaternion new_quaternion = diff_yaw * imu_reading;
	tf::quaternionTFToMsg(new_quaternion, curr_imu_reading.orientation);


	
//	tf::Matrix3x3(new_quaternion).getRPY(n_roll, n_pitch, n_yaw);  
//	ROS_INFO("New RPY:%f,%f,%f",n_roll, n_pitch, n_yaw);



}

void UM6Compass::initFilter(double heading_meas) {
	curr_heading = heading_meas;
	curr_heading_variance = 1; //not very sure
	filter_initialized = true;
}


UM6Compass::UM6Compass() 
{
	//Acquire Parameters
	mag_zero_x = 0; //TODO: Initialize this from a config file
	mag_zero_y = 0;
	mag_zero_z = 0;			

	yaw_meas_variance = 0.1; //TODO: Make this tunable as a parameter

	//Setup Subscribers
	imu_sub_ = n_.subscribe("/imu/data",1000, &UM6Compass::imuCallback, this);
	mag_sub_ = n_.subscribe("/imu/mag",1000, &UM6Compass::magCallback, this);
	imu_pub_ = n_.advertise<sensor_msgs::Imu>("/imu/data_compass",1);
	first_mag_reading = false;
	gyro_update_complete = false;	
	last_motion_update_time = ros::Time::now().toSec();
	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "um6_compass");
	UM6Compass um6_heading_estimator();		
	ros::spin();
	return 0;
}


