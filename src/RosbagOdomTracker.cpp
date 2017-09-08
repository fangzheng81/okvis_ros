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
  // Is this the very first gps message? Set a datum
  if (!this->last_gps_msg) {
    double utm_north, utm_east;
    gps_common::LLtoUTM(msg->latitude, msg->longitude, utm_north, utm_east, utm_zone);
    tf2::Vector3 v{utm_east, utm_north, msg->altitude};
    this->U_p_WU = -v;
    ROS_INFO_STREAM("Set UTM datum " << utm_north << ", " << utm_east << ", " << msg->altitude);
  }

  this->last_gps_msg = msg;
  return true;
}

bool RosbagOdomTracker::consumeAttitudeMsg(const rosbag::MessageInstance &instance) {
  // Try to read QuaternionStamped message or return null if incompatible
  auto msg = instance.instantiate<geometry_msgs::QuaternionStamped>();
  if (!msg) {
    return false;
  }

  // Is this the very first attitude message? Set the datum
  if (!this->last_attitude_msg) {
    // the message is transform from body to ENU ground frame, but at first step world == body
    tf2::Quaternion q_UW;
    tf2::fromMsg(msg->quaternion, q_UW);
    this->q_WU = q_UW.inverse();
  }

  this->last_attitude_msg = msg;
  return true;
}

bool RosbagOdomTracker::consumeVelocityMsg(const rosbag::MessageInstance &instance) {
  // Try to read Vector3Stamped message or return null if incompatible
  auto msg = instance.instantiate<geometry_msgs::Vector3Stamped>();
  if (!msg) {
    return false;
  }
  this->last_velocity_msg = msg;
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

    publishLatest();
  }
}

tf2::Vector3 RosbagOdomTracker::U_p_UB() const {
  double utm_north, utm_east;
  std::string msg_utm_zone;
  gps_common::LLtoUTM(this->last_gps_msg->latitude, last_gps_msg->longitude, utm_north, utm_east, msg_utm_zone);
  if (utm_zone != msg_utm_zone) {
    ROS_ERROR("The UTM zone changed; I can't handle this.");  // unlikely
  }

  return tf2::Vector3{utm_east, utm_north, this->last_gps_msg->altitude};
}

tf2::Quaternion RosbagOdomTracker::q_UB() const {
  tf2::Quaternion q_UB;
  tf2::fromMsg(this->last_attitude_msg->quaternion, q_UB);
  return q_UB;
}


tf2::Vector3 RosbagOdomTracker::U_v_UB() const {
  tf2::Vector3 v;
  tf2::fromMsg(this->last_velocity_msg->vector, v);
  return v;
}

void RosbagOdomTracker::publishLatest() {
  if (!this->initialized()) { return; }

  nav_msgs::Odometry msg;
  msg.header.frame_id = "world";
  tf2::Vector3 W_p_WB = tf2::Matrix3x3{this->q_WU} * (this->U_p_WU + this->U_p_UB());
  tf2::toMsg(W_p_WB, msg.pose.pose.position);
  tf2::Quaternion q_WB = this->q_WU * this->q_UB();
  msg.pose.pose.orientation = tf2::toMsg(q_WB);
  tf2::Vector3 W_v_WB = tf2::Matrix3x3{this->q_WU}  * this->U_v_UB();
  msg.twist.twist.linear = tf2::toMsg(W_v_WB) ;

  this->odom_pub.publish(msg);
}

}  // namespace okvis
