#include "RosbagOdomTracker.hpp"
#include <gps_common/conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace okvis {

RosbagOdomTracker::RosbagOdomTracker(ros::NodeHandle &nh, rosbag::Bag &bag) {
  odom_pub = nh.advertise<nav_msgs::Odometry>("reference_odom", 10);

  std::string gps_topic, attitude_topic, velocity_topic;
  this->active = nh.getParam("gps_topic", gps_topic);
  this->active *= nh.getParam("attitude_topic", attitude_topic);
  this->active *= nh.getParam("velocity_topic", velocity_topic);

  if (active) {
    ROS_INFO("Reading gps messages and publishing odometry");
  } else {
    ROS_INFO("Not publishing odometry since not all of (gps_topic, attitude_topic, velocity_topic) are set");
  }

  this->view.addQuery(bag, rosbag::TopicQuery(gps_topic));
  this->view.addQuery(bag, rosbag::TopicQuery(attitude_topic));
  this->view.addQuery(bag, rosbag::TopicQuery(velocity_topic));
  this->view_iter = this->view.begin();

  this->odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

}

bool RosbagOdomTracker::consumeGpsMsg(const rosbag::MessageInstance &instance) {
  // Try to read NavSatFix message or return null if incompatible
  auto msg = instance.instantiate<sensor_msgs::NavSatFix>();
  if (!msg) {
    return false;
  }

  double utm_north, utm_east;
  gps_common::LLtoUTM(msg->latitude, msg->longitude, utm_north, utm_east, utm_zone);
  tf2::Vector3 v{utm_east, utm_north, msg->altitude};

  // Is this the very first gps message? Set a datum
  if (!this->received_gps_msg) {
    this->U_p_LU = -v;
    ROS_INFO_STREAM("Set UTM datum " << utm_north << ", " << utm_east << ", " << msg->altitude);
  }

  this->last_U_p_UB = v;

  this->received_gps_msg = true;
  return true;
}

bool RosbagOdomTracker::consumeAttitudeMsg(const rosbag::MessageInstance &instance) {
  // Try to read QuaternionStamped message or return null if incompatible
  auto msg = instance.instantiate<geometry_msgs::QuaternionStamped>();
  if (!msg) {
    return false;
  }

  // Is this the very first attitude message? Set the datum
  if (!this->received_attitude_msg) {
    // the message is transform from body to ENU ground frame, but at first step world == body
    tf2::Quaternion q_UW;
    tf2::fromMsg(msg->quaternion, q_UW);
    this->q_WL = q_UW.inverse();
  }

  tf2::fromMsg(msg->quaternion, this->last_q_UB);

  this->received_attitude_msg = true;
  return true;
}

bool RosbagOdomTracker::consumeVelocityMsg(const rosbag::MessageInstance &instance) {
  // Try to read Vector3Stamped message or return null if incompatible
  auto msg = instance.instantiate<geometry_msgs::Vector3Stamped>();
  if (!msg) {
    return false;
  }

  tf2::fromMsg(msg->vector, this->last_U_v_UB);

  this->received_velocity_msg = true;
  return true;
}

void RosbagOdomTracker::processUpTo(const ros::Time &t) {
  ROS_DEBUG_STREAM("Processing up to " << t);
  auto t_msg = view_iter->getTime();
  for (; view_iter->getTime() < t && view_iter != view.end(); ++view_iter) {
    // Consume the message as only one of the following:
    this->consumeGpsMsg(*view_iter);
    this->consumeAttitudeMsg(*view_iter);
    this->consumeVelocityMsg(*view_iter);

    publishTransform();
    publishLatest();
  }
}

void RosbagOdomTracker::publishLatest() {
  if (!this->initialized()) { return; }

    // publish odometry message: local_map to body
  nav_msgs::Odometry msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "local_map";
  msg.child_frame_id = "body";

  tf2::Vector3 W_p_LB = tf2::Matrix3x3{this->q_LU} * (this->U_p_LU + this->last_U_p_UB);
  tf2::toMsg(W_p_LB, msg.pose.pose.position);

  tf2::Quaternion q_LB = this->q_LU * this->last_q_UB;
  msg.pose.pose.orientation = tf2::toMsg(q_LB);

  const auto U_v_LU = tf2::Vector3{0, 0, 0};
  tf2::Vector3 L_v_LB = tf2::Matrix3x3{this->q_LU}  * (U_v_LU + this->last_U_v_UB);
  msg.twist.twist.linear = tf2::toMsg(L_v_LB) ;
  this->odom_pub.publish(msg);
}

void RosbagOdomTracker::publishTransform() {
  if (!initialized()) {
    return;
  }
  // publish transform: local_map to world
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "local_map";

  transformStamped.transform.rotation = tf2::toMsg(this->q_WL);
  transformStamped.transform.translation = tf2::toMsg(this->W_p_WL);

  transformStamped.header.stamp = ros::Time::now();
  this->tf_pub.sendTransform(transformStamped);


  // publish transform: map to local_map
  transformStamped.header.frame_id = "local_map";
  transformStamped.child_frame_id = "map";
  transformStamped.transform.rotation = tf2::toMsg(this->q_LU);
  // Map frame is like U, but at the altitude of L
  auto M_p_LM = this->U_p_LU;
  M_p_LM.setZ(0);

  transformStamped.transform.translation = tf2::toMsg(M_p_LM);
  this->tf_pub.sendTransform(transformStamped);
}

}  // namespace okvis
