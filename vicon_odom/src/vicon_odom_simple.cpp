#include <chrono>
#include <deque>
#include <random>

#include <ros/ros.h>
#include <robot_msgs/OdomNoCov.h>
#include <vicon/Subject.h>
#include <Eigen/Geometry>

static ros::Publisher odom_pub;
static robot_msgs::OdomNoCov odom_msg;

std::chrono::high_resolution_clock::time_point last_time;
float desired_dt; // in seconds.
float dt_warn_thresh_percent;
unsigned int n_avg;

double vel_dt = 0.01;

std::deque<Eigen::Vector3f> pos_buffer;

Eigen::Vector3f last_pos(0.0f, 0.0f, 0.0f);
Eigen::Vector4f last_quat(1.0f, 0.0f, 0.0f, 0.0f);
Eigen::Vector3f last_vel(0.0f, 0.0f, 0.0f);

static void vicon_callback(const vicon::Subject &msg_in) {
  vicon::Subject msg(msg_in);
  vicon::Subject fake_msg;

  static ros::Time t_last_proc = msg.header.stamp;

  double dt = (msg.header.stamp - t_last_proc).toSec();
  t_last_proc = msg.header.stamp;

  auto time_now = std::chrono::high_resolution_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(time_now - last_time).count();
  if (dur > dt_warn_thresh_percent * 1e6 * desired_dt) {
    ROS_WARN("vicon_odom callback dt: %f ms", dur / 1000.0);
  }
  last_time = time_now;

  if (msg.occluded) {
    // Keep going with last message.
    fake_msg.header = msg.header;

    Eigen::Vector3f new_pos = last_pos + last_vel * (vel_dt / n_avg);
    fake_msg.position.x = new_pos(0);
    fake_msg.position.y = new_pos(1);
    fake_msg.position.z = new_pos(2);

    fake_msg.orientation.w = last_quat(0);
    fake_msg.orientation.x = last_quat(1);
    fake_msg.orientation.y = last_quat(2);
    fake_msg.orientation.z = last_quat(3);

    msg = fake_msg;
  }

  last_pos(0) = msg.position.x;
  last_pos(1) = msg.position.y;
  last_pos(2) = msg.position.z;
  last_quat(0) = msg.orientation.w;
  last_quat(1) = msg.orientation.x;
  last_quat(2) = msg.orientation.y;
  last_quat(3) = msg.orientation.z;

  pos_buffer.emplace_back(msg.position.x, msg.position.y, msg.position.z);

  if (pos_buffer.size() < n_avg) {
    return;
  }

  Eigen::Vector3f avg_pos(0.0f, 0.0f, 0.0f);
  for (const auto& pos : pos_buffer) {
    avg_pos += pos / n_avg;
  }
  pos_buffer.clear();

  odom_msg.header.seq = msg.header.seq;
  odom_msg.header.stamp = msg.header.stamp;
  odom_msg.header.frame_id = "world";
  odom_msg.pose.position.x = avg_pos(0);
  odom_msg.pose.position.y = avg_pos(1);
  odom_msg.pose.position.z = avg_pos(2);

  odom_msg.pose.orientation.x = msg.orientation.x;
  odom_msg.pose.orientation.y = msg.orientation.y;
  odom_msg.pose.orientation.z = msg.orientation.z;
  odom_msg.pose.orientation.w = msg.orientation.w;

  // Single step differentiatation for velocity and angular velocity
  static Eigen::Vector3f pos_prev(0.0f, 0.0f, 0.0f);
  static Eigen::Matrix3d R_prev(Eigen::Matrix3d::Identity());
  Eigen::Matrix3d R(Eigen::Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z));
  if (dt > 1e-6) {
    Eigen::Vector3f vel = (avg_pos - pos_prev) / vel_dt;
    last_vel = vel;
    odom_msg.twist.linear.x = vel(0);
    odom_msg.twist.linear.y = vel(1);
    odom_msg.twist.linear.z = vel(2);

    Eigen::Matrix3d R_dot = (R - R_prev) / vel_dt;
    Eigen::Matrix3d w_hat = R_dot * R.transpose();

    odom_msg.twist.angular.x = w_hat(2, 1);
    odom_msg.twist.angular.y = w_hat(0, 2);
    odom_msg.twist.angular.z = w_hat(1, 0);
  }

  pos_prev = avg_pos;
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

  n.param("vicon_pub_dt", vel_dt, 0.01);
  ROS_ASSERT(vel_dt > 0.0);

  dt_warn_thresh_percent = 1.5;
  if (vicon_fps > 150) {
    dt_warn_thresh_percent = 3.0;
  }

  n_avg = static_cast<unsigned int>(std::round(vicon_fps * vel_dt));

  ros::Subscriber odom_sub = n.subscribe("vicon", 1, &vicon_callback, ros::TransportHints().udp());
  odom_pub = n.advertise<robot_msgs::OdomNoCov>("odom", 1);

  ros::spin();
  return 0;
}
