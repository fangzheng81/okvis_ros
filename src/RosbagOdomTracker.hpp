#ifndef OKVIS_ROS_ROSBAGODOMTRACKER_HPP
#define OKVIS_ROS_ROSBAGODOMTRACKER_HPP

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_broadcaster.h>
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
    return active && received_gps_msg && received_attitude_msg && received_velocity_msg;
  }

 protected:
  void publishLatest();
  void publishTransform();

  bool received_gps_msg = false;
  bool received_attitude_msg = false;
  bool received_velocity_msg = false;

  tf2::Vector3 U_p_LU;   ///< transform from utm to local_map
  tf2::Quaternion q_WL;  ///< transform from world to local_map
  tf2::Vector3 W_p_WL{0, 0, 0};  ///< no translation from local_map to world
  const tf2::Quaternion q_LU = tf2::Quaternion::getIdentity();  ///< no rotation from local_map to utm


  tf2::Vector3 last_U_p_UB;
  tf2::Quaternion last_q_LB;
  tf2::Vector3 last_U_v_UB;

  std::string utm_zone;
  bool active;
  rosbag::View view;
  rosbag::View::iterator view_iter;
  ros::Publisher odom_pub;
  tf2_ros::TransformBroadcaster tf_pub;
};

}  // namespace okvis

#endif  // OKVIS_ROS_ROSBAGODOMTRACKER_HPP
