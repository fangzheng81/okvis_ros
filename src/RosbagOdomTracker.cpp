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
    tf2::Vector3 v{utm_north, utm_east, msg->altitude};
    utm_from_world.setOrigin(-v);
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
    tf2::Quaternion q;
    tf2::fromMsg(msg->quaternion, q);
    utm_from_world.setRotation(q);
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

tf2::Vector3 RosbagOdomTracker::utmPoint() const {
  double utm_north, utm_east;
  std::string msg_utm_zone;
  gps_common::LLtoUTM(this->last_gps_msg->latitude, last_gps_msg->longitude, utm_north, utm_east, msg_utm_zone);
  if (utm_zone != msg_utm_zone) {
    ROS_ERROR("The UTM zone changed; I can't handle this.");  // unlikely
  }

  return tf2::Vector3{utm_north, utm_east, this->last_gps_msg->altitude};
}

tf2::Quaternion RosbagOdomTracker::lastOrientation() const {
  tf2::Quaternion q;
  tf2::fromMsg(this->last_attitude_msg->quaternion, q);
  return q;
}


tf2::Vector3 RosbagOdomTracker::lastVelocity() const {
  tf2::Vector3 v;
  tf2::fromMsg(this->last_velocity_msg->vector, v);
  return v;
}

void RosbagOdomTracker::publishLatest() {
  if (!this->initialized()) { return; }

  nav_msgs::Odometry msg;
  msg.header.frame_id = "world";
  tf2::Vector3 world_pos =  this->utm_from_world.getOrigin() + this->utmPoint();
  tf2::toMsg(world_pos, msg.pose.pose.position);
  msg.pose.pose.orientation = tf2::toMsg(this->utm_from_world.getRotation() * this->lastOrientation());
  tf2::Vector3 world_vel = this->utm_from_world.getBasis() * this->lastVelocity();
  msg.twist.twist.linear = tf2::toMsg(world_vel) ;

  this->odom_pub.publish(msg);
}

}  // namespace okvis
