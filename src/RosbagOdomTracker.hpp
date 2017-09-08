#ifndef OKVIS_ROS_ROSBAGODOMTRACKER_HPP
#define OKVIS_ROS_ROSBAGODOMTRACKER_HPP

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2/LinearMath/Transform.h>
#include <rosbag/view.h>

namespace okvis {

/// \brief Reads gps data from a rosbag and publishes odometry in the okvis world frame
class RosbagOdomTracker {
 public:
  RosbagOdomTracker(ros::NodeHandle &nh, rosbag::Bag &bag);

  bool consumeGpsMsg(const rosbag::MessageInstance &instance);
  bool consumeAttitudeMsg(const rosbag::MessageInstance &instance);
  bool consumeVelocityMsg(const rosbag::MessageInstance &instance);

  void processUpTo(const ros::Time &t);

  bool initialized() const {
    return active && last_gps_msg && last_attitude_msg && last_velocity_msg;
  }

 protected:
  tf2::Vector3 U_p_UB() const;
  tf2::Quaternion q_UB() const;
  tf2::Vector3 U_v_UB() const;
  void publishLatest();

  boost::shared_ptr<sensor_msgs::NavSatFix> last_gps_msg;
  boost::shared_ptr<geometry_msgs::QuaternionStamped> last_attitude_msg;
  boost::shared_ptr<geometry_msgs::Vector3Stamped> last_velocity_msg;

  tf2::Vector3 U_p_WU;   ///< transform from utm to world
  tf2::Quaternion q_WU;  ///< transform from utm to world
  std::string utm_zone;
  bool active;
  rosbag::View view;
  rosbag::View::iterator view_iter;
  ros::Publisher odom_pub;
};

}  // namespace okvis

#endif  // OKVIS_ROS_ROSBAGODOMTRACKER_HPP
