#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TwistStamped.h>

#include <math.h>

/** TYPES **/


/* Shorthand to define a subscriber of MotorSpeed messages */
typedef message_filters::Subscriber<robotics_hw1::MotorSpeed> SpinFilterSubscriber;


/* Short version of synch policy */ 
typedef message_filters::sync_policies
	::ApproximateTime<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed>
	MotorSynchPolicy;


struct MotorRpms {
	double fr, fl, rr,  rl;

	double r() const { return (fr + rr)/2; }

	double l() const { return (fl + rl)/2; }
};

struct RobotData {
	double gear, wheelRadius, baseLine;
};


/** GLOBAL **/
static ros::Publisher geometry_pub;


/** CODE **/

// Convet 3 doubles into a vector3
geometry_msgs::Vector3 asVector3(double x, double y, double z)
{
	auto v3 = geometry_msgs::Vector3();
	v3.x = x;
	v3.y = y;
	v3.z = z;

	return v3;
}


inline double rpmToSpeed(double rpm, double gear, double radius)
{
	double wheel_w = rpm / gear * M_PI / 30; // rad/sec
	return wheel_w * radius; // wheel speed
} 


geometry_msgs::Vector3 calcForwardVelocity(const MotorRpms& rpms, const RobotData& robotData) 
{				
	double vr = rpmToSpeed(rpms.r(), robotData.gear, robotData.wheelRadius);
	double vl = rpmToSpeed(rpms.l(), robotData.gear, robotData.wheelRadius);
	double fwd = (vr - vl)/2; 
	return asVector3(fwd, 0, 0);
}


geometry_msgs::Vector3 calcAngularVelocity(const MotorRpms& rpms, const RobotData& robotData)
{
	double vr = rpmToSpeed(rpms.r(), robotData.gear, robotData.wheelRadius);
	double vl = rpmToSpeed(rpms.l(), robotData.gear, robotData.wheelRadius);
	double z = (vr + vl)/ robotData.baseLine;

	return asVector3(0, 0, z);	
}


// Publish a velocity msg
void publishVelocity(geometry_msgs::Vector3 linear, geometry_msgs::Vector3 angular, std::string frame_id ="")
{
	auto msg = geometry_msgs::TwistStamped();

	msg.header.frame_id = frame_id;
	msg.header.stamp = ros::Time::now();

	msg.twist.angular = angular;
	msg.twist.linear = linear;

	geometry_pub.publish(msg);
}


void onMotorSynch(
	const robotics_hw1::MotorSpeedConstPtr& fr,
 	const robotics_hw1::MotorSpeedConstPtr& fl,
	const robotics_hw1::MotorSpeedConstPtr& rr, 
	const robotics_hw1::MotorSpeedConstPtr& rl
	)
{

	MotorRpms rpms = {fr->rpm, fl->rpm, rr->rpm, rl->rpm};
	RobotData robotData = { /*gear*/ 37, /*wheel rad*/	0.1575, /*baseline*/1.0331}; 

	// pass by ref (like pointer without bad math)

	auto fwd = calcForwardVelocity(rpms, robotData);
	auto ang = calcAngularVelocity(rpms, robotData);

	ROS_INFO ("Velocity tick -> Fwd: [%f %f %f], Ang: [%f %f %f]",
		fwd.x, fwd.y, fwd.z, ang.x, ang.y, ang.z
	);

	ROS_INFO ("rpm: [%f, %f, %f, %f]", fr->rpm, fl->rpm, rr->rpm, rl->rpm);

	// send result
	publishVelocity(fwd, ang);
}



int main(int argc, char **argv)
{
    
	ros::init(argc, argv, "rb1_velocity");
	ros::NodeHandle n;

	SpinFilterSubscriber sub_fr_motor(n, "motor_speed_fr", 1);
	SpinFilterSubscriber sub_fl_motor(n, "motor_speed_fl", 1);
	SpinFilterSubscriber sub_rr_motor(n, "motor_speed_rr", 1);
	SpinFilterSubscriber sub_rl_motor(n, "motor_speed_rl", 1);

	// prepare published
	geometry_pub = n.advertise<geometry_msgs::TwistStamped>("velocity_update", 1000);
	
	//TODO: controllare cosa e' quel 10 magico che non mi ricordo
	message_filters::Synchronizer<MotorSynchPolicy> 
		sync(MotorSynchPolicy(10), sub_fr_motor, sub_fl_motor, sub_rr_motor, sub_rl_motor);

  	sync.registerCallback(boost::bind(&onMotorSynch, _1, _2, _3, _4));

	ros::spin();

  	return 0;
}