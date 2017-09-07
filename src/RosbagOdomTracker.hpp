#ifndef OKVIS_ROS_ROSBAGODOMTRACKER_HPP
#define OKVIS_ROS_ROSBAGODOMTRACKER_HPP

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <rosbag/view.h>

namespace okvis {

/// \brief Reads gps data from a rosbag and publishes odometry in the okvis world frame
class RosbagOdomTracker {
 public:
  RosbagOdomTracker(ros::NodeHandle &nh, rosbag::Bag &bag) {
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

  bool consumeGpsMsg(const rosbag::MessageInstance &instance) {
    // Try to read NavSatFix message or return null if incompatible
    auto msg = instance.instantiate<sensor_msgs::NavSatFix>();
    if (!msg) {
      return false;
    }
    // Is this the very first gps message? Set a datum
    if (!this->last_gps_msg) {
      double utm_north, utm_east;
      gps_common::LLtoUTM(msg->latitude, msg->longitude, utm_north, utm_east, utm_zone);
      utm_datum << utm_north, utm_east, msg->altitude;
      ROS_INFO_STREAM("Set UTM datum " << utm_datum.transpose());
    }

    this->last_gps_msg = msg;
    return true;
  }

  bool consumeAttitudeMsg(const rosbag::MessageInstance &instance) {
    // Try to read QuaternionStamped message or return null if incompatible
    auto msg = instance.instantiate<geometry_msgs::QuaternionStamped>();
    if (!msg) {
      return false;
    }
    // Is this the very first attitude message? Set the datum
    if (!this->last_attitude_msg) {
      tf::quaternionMsgToEigen(msg->quaternion, this->q_datum);
    }

    this->last_attitude_msg = msg;
    return true;
  }

  bool consumeVelocityMsg(const rosbag::MessageInstance &instance) {
    // Try to read Vector3Stamped message or return null if incompatible
    auto msg = instance.instantiate<geometry_msgs::Vector3Stamped>();
    if (!msg) {
      return false;
    }
    this->last_velocity_msg = msg;
    return true;
  }

  void processUpTo(const ros::Time &t) {
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

  bool initialized() const {
    return active && last_gps_msg && last_attitude_msg && last_velocity_msg;
  }

  Eigen::Vector3d pointInWorldFrame() const {
    double utm_north, utm_east;
    std::string msg_utm_zone;
    gps_common::LLtoUTM(this->last_gps_msg->latitude, last_gps_msg->longitude, utm_north, utm_east, msg_utm_zone);
    if (utm_zone != msg_utm_zone) {
      ROS_ERROR("The UTM zone changed; I can't handle this.");  // unlikely
    }

    Eigen::Vector3d point;
    // translate to datum
    point.x() = utm_north - utm_datum.x();
    point.y() = utm_east - utm_datum.y();
    point.z() = this->last_gps_msg->altitude - utm_datum.z();

    // rotate according to inital attitude
    point = this->q_datum.inverse() * point;
    return point;
  }

  Eigen::Quaterniond orientationInWorldFrame() const {
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(this->last_attitude_msg->quaternion, q);

    // rotate according to inital attitude
    return this->q_datum.inverse() * q;
  }

  Eigen::Vector3d velocityInWorldFrame() const {
    Eigen::Vector3d vel;
    tf::vectorMsgToEigen(this->last_velocity_msg->vector, vel);
    // rotate according to inital attitude
    return this->q_datum.inverse() * vel;
  }

  void publishLatest() {
    if (!this->initialized()) { return; }

    nav_msgs::Odometry msg;
    msg.header.frame_id = "world";
    tf::pointEigenToMsg(this->pointInWorldFrame(), msg.pose.pose.position);
    tf::quaternionEigenToMsg(this->orientationInWorldFrame(), msg.pose.pose.orientation);
    tf::vectorEigenToMsg(this->velocityInWorldFrame(), msg.twist.twist.linear);

    this->odom_pub.publish(msg);
  }

 protected:
  boost::shared_ptr<sensor_msgs::NavSatFix> last_gps_msg;
  boost::shared_ptr<geometry_msgs::QuaternionStamped> last_attitude_msg;
  boost::shared_ptr<geometry_msgs::Vector3Stamped> last_velocity_msg;

  Eigen::Vector3d utm_datum;
  Eigen::Quaterniond q_datum;
  std::string utm_zone;
  bool active;
  rosbag::View view;
  rosbag::View::iterator view_iter;
  ros::Publisher odom_pub;
};

}  // namespace okvis

#endif  // OKVIS_ROS_ROSBAGODOMTRACKER_HPP
