#include "ros/ros.h"

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <dynamic_reconfigure/server.h>
#include <rb1/parametersConfig.h>

#include "rb1/MOdometry.h"
#include "rb1/ResetPose.h"
#include "rb1/ResetToZeroZero.h"

/** TYPES **/

enum IntegrationMethod { Euler, RungeKutta }; 

/** GLOBAL **/

/** CODE **/

class compute_odometry {
public:
  compute_odometry() {
    lastTime = ros::Time::now();
    n.getParam("/pose/X", initialX);
    n.getParam("/pose/Y", initialY);
    n.getParam("/pose/Theta", initialTheta);
    integMethod = Euler;

    method_callback = boost::bind(&compute_odometry::onIntegrationMethodChange, this, _1, _2);
    method_server.setCallback(method_callback);

    odom_sub = n.subscribe("velocity_update", 1000, &compute_odometry::onVelocityUpdate, this);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
    custom_odom_pub = n.advertise<rb1::MOdometry>("custom_odm", 1000);

    pose_service = n.advertiseService("reset_pose", &compute_odometry::resetOdometryPose, this);
    zero_service = n.advertiseService("reset_to_zero", &compute_odometry::resetOdometryToZero, this);
  }

  void onIntegrationMethodChange(rb1::parametersConfig &config, uint32_t level){
    switch (config.integMethod)
    {
    case 0:
      integMethod = Euler;
      break;
    case 1:
      integMethod = RungeKutta;
      break;
    } 
  }

  void onVelocityUpdate(const geometry_msgs::TwistStampedConstPtr& velocity){

  	//Compute odometry
  	ros::Time currentTime = ros::Time::now();

  	double deltaT = (currentTime - lastTime).toSec();  //TODO check lastTime
    double newX, newY, newTheta;

    switch (integMethod)
    {
    case Euler:
      newX = initialX + velocity->twist.linear.x * deltaT * cos(initialTheta);
      newY = initialY + velocity->twist.linear.x * deltaT * sin(initialTheta);
      newTheta = initialTheta + velocity->twist.angular.z * deltaT;
      break;
    case RungeKutta:
      newX = initialX + velocity->twist.linear.x * deltaT * cos(initialTheta + velocity->twist.angular.z * deltaT / 2);
      newY = initialY + velocity->twist.linear.x * deltaT * sin(initialTheta + velocity->twist.angular.z * deltaT / 2);
      newTheta = initialTheta + velocity->twist.angular.z * deltaT;
      break;
    }

  	//Publish result
    publishTFOdometry(newX, newY, newTheta, currentTime);
    publishOdometryMsg(newX, newY, newTheta, currentTime, velocity->twist.linear.x, velocity->twist.linear.y, velocity->twist.angular.z);
  	
  	lastTime = currentTime;
    initialX = newX;
    initialY = newY;
    initialTheta = newTheta;
  }

  //Publish the transform over tf (Determine the robot's location in the world and relate sensor data to a static map)
  void publishTFOdometry(double x, double y, double theta, ros::Time time){

  	odomTransform.header.stamp = time;
    odomTransform.header.frame_id = "odom";
    odomTransform.child_frame_id = "base_link";

    //Set the position
    odomTransform.transform.translation.x = x;
    odomTransform.transform.translation.y = y;
    odomTransform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);

    odomTransform.transform.rotation.x = q.x();
    odomTransform.transform.rotation.y = q.y();
    odomTransform.transform.rotation.z = q.z();
    odomTransform.transform.rotation.w = q.w();

    //Send the transform
    br.sendTransform(odomTransform);
  }

  //Publish the odometry message over ROS (Provide information about the velocity of the robot)
  void publishOdometryMsg(double x, double y, double theta, ros::Time time, double vx, double vy, double omega){
    
  	odomMsg.header.stamp = time;
  	odomMsg.header.frame_id = "odom";

  	//Set the position
  	odomMsg.pose.pose.position.x = x;
  	odomMsg.pose.pose.position.y = y;
  	odomMsg.pose.pose.position.z = 0.0;

  	tf2::Quaternion q;
    q.setRPY(0, 0, theta);

    odomMsg.pose.pose.orientation.x = q.x();
    odomMsg.pose.pose.orientation.y = q.y();
    odomMsg.pose.pose.orientation.z = q.z();
    odomMsg.pose.pose.orientation.w = q.w();

  	//Set the velocity
  	odomMsg.child_frame_id = "base_link";
  	odomMsg.twist.twist.linear.x = vx;
  	odomMsg.twist.twist.linear.y = vy;
  	odomMsg.twist.twist.angular.z = omega;

  	odom_pub.publish(odomMsg);
    
    publishCustomOdometryMsg(odomMsg, integMethod);
  }

  bool resetOdometryPose(rb1::ResetPose::Request &req, rb1::ResetPose::Response &res){
    initialX = req.givenX;
    initialY = req.givenY;
    initialTheta = req.givenTheta;
    lastTime = ros::Time::now();
    return true;
  }

  bool resetOdometryToZero(rb1::ResetPose::Request &req, rb1::ResetPose::Response &res){
    initialX = 0.0;
    initialY = 0.0;
    lastTime = ros::Time::now();
    return true;
  }

private:

  void publishCustomOdometryMsg(nav_msgs::Odometry odom, IntegrationMethod method)
  {
    auto msg = rb1::MOdometry();
    msg.odom = odom;
    msg.method = std_msgs::String();

    switch (method)
    {
    case Euler:
      msg.method.data = "euler";
      break;
    case RungeKutta:
      msg.method.data = "rk";
      break;
    }

    custom_odom_pub.publish(msg);
  }

  ros::NodeHandle n; 
  tf2_ros::TransformBroadcaster br;

  // i msg possiamo reistanziarli ogni volta
  geometry_msgs::TransformStamped odomTransform;
  nav_msgs::Odometry odomMsg;

  ros::Subscriber odom_sub;
  ros::Publisher odom_pub;
  ros::Publisher custom_odom_pub;
  ros::ServiceServer pose_service;
  ros::ServiceServer zero_service;

  dynamic_reconfigure::Server<rb1::parametersConfig> method_server;
  dynamic_reconfigure::Server<rb1::parametersConfig>::CallbackType method_callback;
  IntegrationMethod integMethod;

  ros::Time lastTime;
  double initialX;
  double initialY;
  double initialTheta;
};

int main(int argc, char **argv){
    
	ros::init(argc, argv, "rb1_odometry");

	compute_odometry my_odometry;

  ros::spin();

  return 0;
}