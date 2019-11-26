#include <chrono>
#include <random>

#include <ros/ros.h>
#include <robot_msgs/OdomNoCov.h>
#include <vicon/Subject.h>
#include <Eigen/Geometry>

static ros::Publisher odom_pub;
static robot_msgs::OdomNoCov odom_msg;

std::chrono::high_resolution_clock::time_point last_time;
float desired_dt; // in seconds.

static void vicon_callback(const vicon::Subject::ConstPtr &msg) {
  static ros::Time t_last_proc = msg->header.stamp;

  double dt = (msg->header.stamp - t_last_proc).toSec();
  t_last_proc = msg->header.stamp;

  auto time_now = std::chrono::high_resolution_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(time_now - last_time).count();
  if (dur > 1.5 * 1e6 * desired_dt) {
    ROS_WARN("vicon_odom callback dt: %f ms", dur / 1000.0);
  }
  last_time = time_now;

  Eigen::Vector3f pos(msg->position.x, msg->position.y, msg->position.z);

  odom_msg.header.seq = msg->header.seq;
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = "world";
  odom_msg.pose.position.x = pos(0);
  odom_msg.pose.position.y = pos(1);
  odom_msg.pose.position.z = pos(2);

  odom_msg.pose.orientation.x = msg->orientation.x;
  odom_msg.pose.orientation.y = msg->orientation.y;
  odom_msg.pose.orientation.z = msg->orientation.z;
  odom_msg.pose.orientation.w = msg->orientation.w;

  // Single step differentiatation for velocity and angular velocity
  static Eigen::Vector3f pos_prev(0.0, 0.0, 0.0);
  static Eigen::Matrix3d R_prev(Eigen::Matrix3d::Identity());
  Eigen::Matrix3d R(Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z));
  if (dt > 1e-6) {
    Eigen::Vector3f vel = (pos - pos_prev) / dt;
    odom_msg.twist.linear.x = vel(0);
    odom_msg.twist.linear.y = vel(1);
    odom_msg.twist.linear.z = vel(2);

    Eigen::Matrix3d R_dot = (R - R_prev) / dt;
    Eigen::Matrix3d w_hat = R_dot * R.transpose();

    odom_msg.twist.angular.x = w_hat(2, 1);
    odom_msg.twist.angular.y = w_hat(0, 2);
    odom_msg.twist.angular.z = w_hat(1, 0);
  }

  pos_prev = pos;
  R_prev = R;

  odom_pub.publish(odom_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vicon_odom");
  ros::NodeHandle n("~");

  double vicon_fps;
  n.param("vicon_fps", vicon_fps, 100.0);
  ROS_ASSERT(vicon_fps > 0.0);
  desired_dt = 1 / vicon_fps;

  ros::Subscriber odom_sub = n.subscribe("vicon", 1, &vicon_callback, ros::TransportHints().udp());
  odom_pub = n.advertise<robot_msgs::OdomNoCov>("odom", 1);

  ros::spin();
  return 0;
}
