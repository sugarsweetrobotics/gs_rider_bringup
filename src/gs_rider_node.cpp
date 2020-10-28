
#include <memory>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

#include "gs_rider_bringup/gs_rider.h"
#include "gs_rider_bringup/gs_rider_sim.h"
#include "gs_rider_bringup/gs_rider_kinematics.h"
#include "gs_rider_bringup/gs_rider_impl.h"
using namespace gs;


int main(int argc, char* argv[]) {

  ros::init(argc, argv, "gs_rider");
  ros::NodeHandle nodeHandle;
  
  double L = 1.0;
  double init_x, init_y, init_th;
  double loopHz = 10.0;
  std::string base_link_name = "base_link";
  std::string use_sim = "true";
  nodeHandle.param<double>("rider_length", L, 1.0);
  nodeHandle.param<double>("control_rate", loopHz, 10.0);
  nodeHandle.param<double>("init_x", init_x, 0.0);
  nodeHandle.param<double>("init_y", init_y, 0.0);
  nodeHandle.param<double>("init_th", init_th, 0.0);
  nodeHandle.param<std::string>("base_link_name", base_link_name, "base_link");
  nodeHandle.param<std::string>("use_sim", use_sim, "false");

  
  std::shared_ptr<GSRider> gsRider;
  if (use_sim == "true") {
    gsRider = std::make_shared<GSRider>(L, std::make_shared<GSRiderSim>());
  } else {
    gsRider = std::make_shared<GSRider>(L, std::make_shared<GSRiderImpl>(&nodeHandle));
  }
  
  OdometryAccumulator odomAcc(init_x, init_y, init_th);

  // Velocity command callback
  boost::function<void (const geometry_msgs::Twist&)> velCB = [&gsRider] (const auto& msg) { 
    ROS_INFO("I heard: [%f, %f, %f]", msg.linear.x, msg.linear.y, msg.angular.z);
    gsRider->setVelocity(msg.linear.x, msg.angular.z);
  };

  // Transform Broadcast
  tf::TransformBroadcaster br;
  auto tfBroadCast = [&br, &base_link_name] (const std::tuple<double, double, double>& pose) {
    double x, y, th;
    std::tie(x, y, th) = pose;
    //ROS_INFO("ODOM(%f, %f, %f)", x, y, th);
    br.sendTransform(tf::StampedTransform(tf::Transform{tf::Quaternion{{0,0,1}, th}, tf::Vector3{x, y, 0.0}}, ros::Time::now(), "odom", base_link_name));
  };

  //ros::Publisher odomPub = n.advertise<std_msgs::String>("chatter", 1000);  
  ros::Subscriber velocitySub = nodeHandle.subscribe<geometry_msgs::Twist>("velocity", 1000, velCB);

  ros::Rate loop_rate(loopHz);
  
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    tfBroadCast(odomAcc.push(gsRider->update(1.0/loopHz)));
    ROS_INFO("Core: [%f, %f]", gsRider->getSteerAngle(), gsRider->getMoveVelocity());
  }
  
};
