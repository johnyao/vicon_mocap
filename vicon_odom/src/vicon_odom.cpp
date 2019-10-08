#include <chrono>
#include <random>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <robot_msgs/OdomNoCov.h>
#include <vicon/Subject.h>
#include <vicon_odom/filter.h>
#include <Eigen/Geometry>
#include <tf2_ros/transform_broadcaster.h>

static ros::Publisher odom_pub;
static ros::Publisher pose_cov_pub;
static ros::Publisher pose_pub;
static KalmanFilter kf;
static nav_msgs::Odometry odom_msg;
static geometry_msgs::PoseStamped pose_msg;
static geometry_msgs::PoseWithCovarianceStamped pose_cov_msg;
static robot_msgs::OdomNoCov odom_nocov_msg;
static tf2_ros::TransformBroadcaster* tfb;

static bool publish_nocov_odom;

static std::string fixed_frame_id;
static std::string base_frame_id;

static KalmanFilter::State_t proc_noise_diag;
static KalmanFilter::Measurement_t meas_noise_diag;
static unsigned int consecutive_occlusions;

// the minimum number of consecutive time steps at which less than 3 markers are visible before the KF can be re-initialized
static unsigned int min_consec_occ;

// the minimum number of markers that must be visible for vicon attitude to be published
static unsigned int min_visible_markers;

// Variance of the noise added to the position pose message.
static float pos_injected_noise_stddev = 0.0;
// Variance of the extra noise added to the position pose message in between
// the given bounds.
static float pos_injected_extra_noise_stddev = 0.0;
static float high_noise_x_min = -FLT_MAX;
static float high_noise_y_min = -FLT_MAX;
static float high_noise_z_min = -FLT_MAX;
static float high_noise_x_max = FLT_MAX;
static float high_noise_y_max = FLT_MAX;
static float high_noise_z_max = FLT_MAX;
static bool apply_position_noise = false;
static bool apply_extra_position_noise = false;

std::default_random_engine *generator;
std::normal_distribution<float> *normal_dist;

#define OPTIONAL_GET(param) \
  if (n.hasParam(#param)) \
  { \
    n.getParam(#param, param); \
  }

std::chrono::high_resolution_clock::time_point last_time;
float desired_dt; // in seconds.

static void vicon_callback(const vicon::Subject::ConstPtr &msg)
{
  static ros::Time t_last_proc = msg->header.stamp;

  double dt = (msg->header.stamp - t_last_proc).toSec();
  t_last_proc = msg->header.stamp;

  auto time_now = std::chrono::high_resolution_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(time_now - last_time).count();
  if (dur > 1.5 * 1e6 * desired_dt) {
    ROS_WARN("vicon_odom callback dt: %f ms", dur / 1000.0);
  }
  last_time = time_now;

  unsigned int num_visible_markers = 0;
  unsigned int total_num_markers = msg->markers.size();
  for (unsigned int i = 0; i < total_num_markers; i++)
  {
    if (!msg->markers[i].occluded)
      num_visible_markers++;
  }

  if (num_visible_markers < 3)
  {
    //ROS_WARN("vicon_odom: less than 3 visible markers, skipping KF update");
    consecutive_occlusions++;
    return;
  }

  if (consecutive_occlusions > min_consec_occ)
  {
    KalmanFilter::State_t newstate;
    newstate(0) = msg->position.x;
    newstate(1) = msg->position.y;
    newstate(2) = msg->position.z;
    newstate(3) = 0.0;
    newstate(4) = 0.0;
    newstate(5) = 0.0;

    kf.initialize(newstate,
                  0.01*KalmanFilter::ProcessCov_t::Identity(),
                  proc_noise_diag.asDiagonal(),
                  meas_noise_diag.asDiagonal());
    consecutive_occlusions = 0;
    //ROS_WARN("vicon_odom: re-initialize KF position to vicon marker centroid");
  }

  // Kalman filter for getting translational velocity from position measurements
  kf.processUpdate(dt);
  const KalmanFilter::Measurement_t meas(msg->position.x, msg->position.y, msg->position.z);
  if(!msg->occluded)
  {
    static ros::Time t_last_meas = msg->header.stamp;
    double meas_dt = (msg->header.stamp - t_last_meas).toSec();
    t_last_meas = msg->header.stamp;
    kf.measurementUpdate(meas, meas_dt);
  }

  const KalmanFilter::State_t state = kf.getState();
  const KalmanFilter::ProcessCov_t proc_noise = kf.getProcessNoise();

  odom_msg.header.seq = msg->header.seq;
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = fixed_frame_id;
  odom_msg.child_frame_id = base_frame_id;
  odom_msg.pose.pose.position.x = state(0);
  odom_msg.pose.pose.position.y = state(1);
  odom_msg.pose.pose.position.z = state(2);
  odom_msg.twist.twist.linear.x = state(3);
  odom_msg.twist.twist.linear.y = state(4);
  odom_msg.twist.twist.linear.z = state(5);

  for(int i = 0; i < 3; i++)
  {
    for(int j = 0; j < 3; j++)
    {
      odom_msg.pose.covariance[6*i+j] = proc_noise(i,j);
      odom_msg.twist.covariance[6*i+j] = proc_noise(3+i, 3+j);
    }
  }

  pose_cov_msg.header = odom_msg.header;
  pose_cov_msg.pose.pose.position = odom_msg.pose.pose.position;
  pose_cov_msg.pose.pose.orientation = msg->orientation;

  // This should be the variance of the noise present in the raw vicon data.
  float variance = 0.01 * 0.01;

  if (apply_position_noise) {
    pose_cov_msg.pose.pose.position.x += pos_injected_noise_stddev *
                                     normal_dist->operator()(*generator);
    pose_cov_msg.pose.pose.position.y += pos_injected_noise_stddev *
                                     normal_dist->operator()(*generator);
    pose_cov_msg.pose.pose.position.z += pos_injected_noise_stddev *
                                     normal_dist->operator()(*generator);
    variance += std::pow(pos_injected_noise_stddev, 2);
  }

  if (apply_extra_position_noise)
  {
    if (high_noise_x_min < odom_msg.pose.pose.position.x &&
        odom_msg.pose.pose.position.x < high_noise_x_max &&
        high_noise_y_min < odom_msg.pose.pose.position.y &&
        odom_msg.pose.pose.position.y < high_noise_y_max &&
        high_noise_z_min < odom_msg.pose.pose.position.z &&
        odom_msg.pose.pose.position.z < high_noise_z_max)
    {
      pose_cov_msg.pose.pose.position.x += pos_injected_extra_noise_stddev *
                                       normal_dist->operator()(*generator);
      pose_cov_msg.pose.pose.position.y += pos_injected_extra_noise_stddev *
                                       normal_dist->operator()(*generator);
      pose_cov_msg.pose.pose.position.z += pos_injected_extra_noise_stddev *
                                       normal_dist->operator()(*generator);
      variance += std::pow(pos_injected_extra_noise_stddev, 2);
    }
  }

  pose_cov_msg.pose.covariance[0] = variance;
  pose_cov_msg.pose.covariance[7] = variance;
  pose_cov_msg.pose.covariance[14] = variance;
  pose_cov_msg.pose.covariance[35] = 0.02 * 0.02;

  odom_msg.pose.covariance[0] = variance;
  odom_msg.pose.covariance[7] = variance;
  odom_msg.pose.covariance[14] = variance;
  odom_msg.pose.covariance[35] = 0.02 * 0.02;

  pose_cov_pub.publish(pose_cov_msg);

  pose_msg.header = pose_cov_msg.header;
  pose_msg.pose = pose_cov_msg.pose.pose;
  pose_pub.publish(pose_msg);

  if (num_visible_markers >= min_visible_markers)
  {
    odom_msg.pose.pose.orientation.x = msg->orientation.x;
    odom_msg.pose.pose.orientation.y = msg->orientation.y;
    odom_msg.pose.pose.orientation.z = msg->orientation.z;
    odom_msg.pose.pose.orientation.w = msg->orientation.w;

    // Single step differentiatation for angular velocity
    static Eigen::Matrix3d R_prev(Eigen::Matrix3d::Identity());
    Eigen::Matrix3d R(Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z));
    if(dt > 1e-6)
    {
      Eigen::Matrix3d R_dot = (R - R_prev)/dt;
      Eigen::Matrix3d w_hat = R_dot * R.transpose();

      odom_msg.twist.twist.angular.x = w_hat(2, 1);
      odom_msg.twist.twist.angular.y = w_hat(0, 2);
      odom_msg.twist.twist.angular.z = w_hat(1, 0);
    }
    R_prev = R;

    geometry_msgs::TransformStamped ts;
    ts.transform.translation.x = odom_msg.pose.pose.position.x;
    ts.transform.translation.y = odom_msg.pose.pose.position.y;
    ts.transform.translation.z = odom_msg.pose.pose.position.z;
    ts.transform.rotation.x = odom_msg.pose.pose.orientation.x;
    ts.transform.rotation.y = odom_msg.pose.pose.orientation.y;
    ts.transform.rotation.z = odom_msg.pose.pose.orientation.z;
    ts.transform.rotation.w = odom_msg.pose.pose.orientation.w;
    ts.header = odom_msg.header;
    ts.child_frame_id = odom_msg.child_frame_id;
    tfb->sendTransform(ts);
  }
  else
  {
    // use invalid quaternion to indicate that attitude is unreliable
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 0.0;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;
  }

  odom_nocov_msg.header = odom_msg.header;
  odom_nocov_msg.pose = odom_msg.pose.pose;
  odom_nocov_msg.twist = odom_msg.twist.twist;

  if (!publish_nocov_odom)
  {
    odom_pub.publish(odom_msg);
  }
  else
  {
    odom_pub.publish(odom_nocov_msg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vicon_odom");

  ros::NodeHandle n("~");

  tfb = new tf2_ros::TransformBroadcaster();

  if (!n.hasParam("frame_id/fixed"))
  {
    ROS_ERROR("vicon_odom: failed to find param 'frame_id/fixed'");
    return EXIT_FAILURE;
  }
  n.getParam("frame_id/fixed", fixed_frame_id);

  if (!n.hasParam("frame_id/base"))
  {
    ROS_ERROR("vicon_odom: failed to find param 'frame_id/base'");
    return EXIT_FAILURE;
  }
  n.getParam("frame_id/base", base_frame_id);

  if (!n.hasParam("vicon_kf/min_consecutive_occlusions_for_restart"))
  {
    ROS_ERROR("vicon_odom: failed to find param 'vicon_kf/min_consecutive_occlusions_for_restart'");
    return EXIT_FAILURE;
  }
  int tmp;
  n.getParam("vicon_kf/min_consecutive_occlusions_for_restart", tmp);
  min_consec_occ = static_cast<unsigned int>(tmp);

  if (!n.hasParam("vicon_kf/min_visible_markers"))
  {
    ROS_ERROR("vicon_odom: failed to find param 'vicon_kf/min_visible_markers'");
    return EXIT_FAILURE;
  }
  n.getParam("vicon_kf/min_visible_markers", tmp);
  min_visible_markers = static_cast<unsigned int>(tmp);

  if (n.hasParam("pos_injected_noise_stddev"))
  {
    apply_position_noise = true;
    n.getParam("pos_injected_noise_stddev", pos_injected_noise_stddev);
  }

  if (n.hasParam("pos_injected_extra_noise_stddev"))
  {
    apply_extra_position_noise = true;
    n.getParam("pos_injected_extra_noise_stddev", pos_injected_extra_noise_stddev);
    OPTIONAL_GET(high_noise_x_min)
    OPTIONAL_GET(high_noise_y_min)
    OPTIONAL_GET(high_noise_z_min)
    OPTIONAL_GET(high_noise_x_max)
    OPTIONAL_GET(high_noise_y_max)
    OPTIONAL_GET(high_noise_z_max)
  }

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine gen(seed);
  std::normal_distribution<float> std_normal(0.0, 1.0);

  // There has to be a better way to do this...
  generator = &gen;
  normal_dist = &std_normal;

  if (min_visible_markers < 4)
  {
    ROS_ERROR("vicon_odom: 'vicon_kf/min_visible_markers' must be at least 4");
    return EXIT_FAILURE;
  }

  double max_accel;
  n.param("max_accel", max_accel, 5.0);

  double dt, vicon_fps;
  n.param("vicon_fps", vicon_fps, 100.0);
  ROS_ASSERT(vicon_fps > 0.0);
  dt = 1/vicon_fps;
  desired_dt = dt;

  publish_nocov_odom = false;
  if (n.hasParam("publish_nocov_odom"))
  {
    n.getParam("publish_nocov_odom", publish_nocov_odom);
  }

  proc_noise_diag(0) = 0.5*max_accel*dt*dt;
  proc_noise_diag(1) = 0.5*max_accel*dt*dt;
  proc_noise_diag(2) = 0.5*max_accel*dt*dt;
  proc_noise_diag(3) = max_accel*dt;
  proc_noise_diag(4) = max_accel*dt;
  proc_noise_diag(5) = max_accel*dt;
  proc_noise_diag = proc_noise_diag.array().square();
  meas_noise_diag(0) = 1e-4;
  meas_noise_diag(1) = 1e-4;
  meas_noise_diag(2) = 1e-4;
  meas_noise_diag = meas_noise_diag.array().square();
  kf.initialize(KalmanFilter::State_t::Zero(),
                0.01*KalmanFilter::ProcessCov_t::Identity(),
                proc_noise_diag.asDiagonal(),
                meas_noise_diag.asDiagonal());

  ros::Subscriber vicon_sub = n.subscribe("vicon", 10, &vicon_callback,
                                          ros::TransportHints().tcpNoDelay());

  if (!publish_nocov_odom)
  {
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
  }
  else
  {
    odom_pub = n.advertise<robot_msgs::OdomNoCov>("odom", 10);
  }

  pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 10);
  pose_cov_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_cov", 10);

  consecutive_occlusions = 0;

  ros::spin();

  return 0;
}
