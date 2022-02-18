/*
 * This is a unstable estimator of the robot baseline. 
 * You should run this node multiple times. The result will likely float around an interval (in our case 0.9-1).
 * Fine tuning by hand is required to obtain good results. Do not blindly trust the first result obtained.
 *	
 * The estimation can be really poor probably because there are some approximation errors made by the syncronizers
 * that leads to different estimations of the baseline in different runs.
 * 
 * Notice that we keep the number of ticks small to use good approximated angular velocities of the odometry.
 */

#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cstdlib>
#include <math.h>


/** TYPES **/


/* Shorthand to define a subscriber of MotorSpeed messages */
typedef message_filters::Subscriber<robotics_hw1::MotorSpeed> SpinFilterSubscriber;

typedef message_filters::Subscriber<nav_msgs::Odometry> OdomFilterSubscriber;


/* Short version of synch policy */ 
typedef message_filters::sync_policies
	::ApproximateTime<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, nav_msgs::Odometry>
	MotorSynchPolicy;


/** GLOBAL **/
static bool finished;
static int ticks = 0;
static float baseline_sum;
static double rpmDiff = 0;
static int ntick = 10;
static int nGiri = 0;
static double prec_y = 0;

static double diffAr = 0;
static double precYaw;
static double anAn;

static double y[512];
static int i = 0;
static double initAn = 0;
static double precX, precY;
static double lenS = 0;
static double sumRpm = 0;

/** CODE **/


void onTick(
	const robotics_hw1::MotorSpeedConstPtr& fr,
 	const robotics_hw1::MotorSpeedConstPtr& fl,
	const robotics_hw1::MotorSpeedConstPtr& rr, 
	const robotics_hw1::MotorSpeedConstPtr& rl,
	const nav_msgs::OdometryConstPtr& expected
	)
{


	rpmDiff = rpmDiff + fr->rpm + fl->rpm + rr->rpm + rl->rpm;
	sumRpm += fr->rpm - fl->rpm + rr->rpm - rl->rpm;

	ticks++;
	tf2::Quaternion q (expected->pose.pose.orientation.x, expected->pose.pose.orientation.y, expected->pose.pose.orientation.z, expected->pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw, yawr;
	m.getRPY(roll, pitch, yawr);
	if(prec_y >2 && yawr< -2){
		nGiri --;
	}else if(prec_y< -2 && yawr > 2){
		nGiri ++;
	}
	yaw = yawr - 2*nGiri*M_PI - initAn;
	if(ticks%ntick == 0){
		if(rpmDiff == 0){
			ROS_INFO("Rpm diff = 0.. not estimating baseline");
			initAn = yawr;
			
		}else if(yaw == 0){
			ROS_INFO("Angole = 0... not estimating baseline");
		}else{

			double rpm2 = 0.1575*M_PI/120;
			double rpm2s= 0.1575/ 37 * M_PI / 30;
			double blf =  rpmDiff/yaw;
			double bl = 2*rpm2s*blf;
			ROS_INFO("baseline_factor : %f",blf);
			ROS_INFO("baseline: %f", bl);
			ROS_INFO("angolo: %f, diff: %f, rpm2s: %f", yaw, rpmDiff, rpm2s);
		}
		
	}
	precX = expected->pose.pose.position.x;
	precY = expected->pose.pose.position.y;
	prec_y = yawr;
	return;

}



int main(int argc, char **argv)
{
    
	ros::init(argc, argv, "rb1_baseline");
	ros::NodeHandle n;
	finished = false;
	ticks = 0;
	baseline_sum = 0;

	SpinFilterSubscriber sub_fr_motor(n, "motor_speed_fr", 1);
	SpinFilterSubscriber sub_fl_motor(n, "motor_speed_fl", 1);
	SpinFilterSubscriber sub_rr_motor(n, "motor_speed_rr", 1);
	SpinFilterSubscriber sub_rl_motor(n, "motor_speed_rl", 1);
	// expected odometry to track
	OdomFilterSubscriber sub_desired_odm(n, "scout_odom", 1);

	
	//TODO: controllare cosa e' quel 10 magico che non mi ricordo
	message_filters::Synchronizer<MotorSynchPolicy> 
		sync(MotorSynchPolicy(10), sub_fr_motor, sub_fl_motor, sub_rr_motor, sub_rl_motor, sub_desired_odm);

  	sync.registerCallback(boost::bind(&onTick, _1, _2, _3, _4, _5));

	ros::spin();

  	return 0;
}