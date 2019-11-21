/* includes //{ */

#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mutex>

#include <nav_msgs/Odometry.h>

#include <mrs_lib/ParamLoader.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

//}

/* defines //{ */

#define STRING_EQUAL 0

//}

namespace mrs_general
{

namespace tomasuv_tf_publisher
{

/* class TomasuvTfPublisher //{ */

class TomasuvTfPublisher : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  std::string     uav_name_;
  bool            is_initialized = false;

private:
  ros::Subscriber subscriber_odom_main;
  ros::Subscriber subscriber_odom_brick;
  ros::Subscriber subscriber_odom_stable;
  ros::Subscriber subscriber_odom_optflow;

private:
  tf2_ros::TransformBroadcaster* tf_broadcaster;

private:
  ros::Timer main_timer;
  void       mainTimer(const ros::TimerEvent& event);
  double     main_timer_rate_;

private:
  void               callbackOdomMain(const nav_msgs::OdometryConstPtr& msg);
  std::mutex         mutex_odom_main;
  nav_msgs::Odometry odom_main;
  bool               got_odom_main = false;

private:
  void               callbackOdomBrick(const nav_msgs::OdometryConstPtr& msg);
  std::mutex         mutex_odom_brick;
  nav_msgs::Odometry odom_brick;
  bool               got_odom_brick = false;

private:
  void               callbackOdomOptflow(const nav_msgs::OdometryConstPtr& msg);
  std::mutex         mutex_odom_optflow;
  nav_msgs::Odometry odom_optflow;
  bool               got_odom_optflow = false;

private:
  void               callbackOdomStable(const nav_msgs::OdometryConstPtr& msg);
  std::mutex         mutex_odom_stable;
  nav_msgs::Odometry odom_stable;
  bool               got_odom_stable = false;
};

//}

/* inInit() //{ */

void TomasuvTfPublisher::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "TomasuvTfPublisher");

  param_loader.load_param("uav_name", uav_name_);
  param_loader.load_param("main_timer_rate", main_timer_rate_);

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[TomasufTfPublisher]: Could not load all parameters!");
    ros::shutdown();
  }

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_odom_main    = nh_.subscribe("odom_main_in", 1, &TomasuvTfPublisher::callbackOdomMain, this, ros::TransportHints().tcpNoDelay());
  subscriber_odom_stable  = nh_.subscribe("odom_stable_in", 1, &TomasuvTfPublisher::callbackOdomStable, this, ros::TransportHints().tcpNoDelay());
  subscriber_odom_brick   = nh_.subscribe("odom_brick_in", 1, &TomasuvTfPublisher::callbackOdomBrick, this, ros::TransportHints().tcpNoDelay());
  subscriber_odom_optflow = nh_.subscribe("odom_optflow_in", 1, &TomasuvTfPublisher::callbackOdomOptflow, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  tf_broadcaster = new tf2_ros::TransformBroadcaster();

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  main_timer = nh_.createTimer(ros::Rate(main_timer_rate_), &TomasuvTfPublisher::mainTimer, this);

  is_initialized = true;

  ROS_INFO("[TomasuvTfPublisher]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackOdomMain() //{ */

void TomasuvTfPublisher::callbackOdomMain(const nav_msgs::OdometryConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  std::scoped_lock lock(mutex_odom_main);

  ROS_INFO_ONCE("[TomasuvTfPublisher]: getting odom main");

  got_odom_main = true;

  odom_main = *msg;
}

//}

/* callbackOdomStable() //{ */

void TomasuvTfPublisher::callbackOdomStable(const nav_msgs::OdometryConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  std::scoped_lock lock(mutex_odom_stable);

  ROS_INFO_ONCE("[TomasuvTfPublisher]: getting odom stable");

  got_odom_stable = true;

  odom_stable = *msg;
}

//}

/* callbackOdomOptflow() //{ */

void TomasuvTfPublisher::callbackOdomOptflow(const nav_msgs::OdometryConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  std::scoped_lock lock(mutex_odom_optflow);

  ROS_INFO_ONCE("[TomasuvTfPublisher]: getting odom optflow");

  got_odom_optflow = true;

  odom_optflow = *msg;
}

//}

/* callbackOdomBrick() //{ */

void TomasuvTfPublisher::callbackOdomBrick(const nav_msgs::OdometryConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  std::scoped_lock lock(mutex_odom_brick);

  ROS_INFO_ONCE("[TomasuvTfPublisher]: getting odom brick");

  got_odom_brick = true;

  odom_brick = *msg;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* mainTimer() //{ */

void TomasuvTfPublisher::mainTimer([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO_ONCE("[TomasuvTfPublisher]: main timer started");

  if (got_odom_brick) {

    std::scoped_lock lock(mutex_odom_brick);

    geometry_msgs::TransformStamped tf;
    tf.header.stamp    = odom_brick.header.stamp;
    tf.header.frame_id = uav_name_ + "/fcu";
    tf.child_frame_id  = uav_name_ + "/brick_origin";

    tf.transform.translation.x = -odom_brick.pose.pose.position.x;
    tf.transform.translation.y = -odom_brick.pose.pose.position.y;
    tf.transform.translation.z = -odom_brick.pose.pose.position.z;

    tf2::Quaternion quat(odom_brick.pose.pose.orientation.x, odom_brick.pose.pose.orientation.y, odom_brick.pose.pose.orientation.z,
                         odom_brick.pose.pose.orientation.w);
    quat = quat.inverse();

    tf.transform.rotation.x = quat.x();
    tf.transform.rotation.y = quat.y();
    tf.transform.rotation.z = quat.z();
    tf.transform.rotation.w = quat.w();

    try {
      tf_broadcaster->sendTransform(tf);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing TF: %s - %s.", tf.child_frame_id.c_str(), tf.header.frame_id.c_str());
    }
  }

  if (got_odom_optflow) {

    std::scoped_lock lock(mutex_odom_optflow);

    geometry_msgs::TransformStamped tf;
    tf.header.stamp    = odom_optflow.header.stamp;
    tf.header.frame_id = uav_name_ + "/fcu";
    tf.child_frame_id  = uav_name_ + "/optflow_origin";

    tf.transform.translation.x = -odom_optflow.pose.pose.position.x;
    tf.transform.translation.y = -odom_optflow.pose.pose.position.y;
    tf.transform.translation.z = -odom_optflow.pose.pose.position.z;

    tf2::Quaternion quat(odom_optflow.pose.pose.orientation.x, odom_optflow.pose.pose.orientation.y, odom_optflow.pose.pose.orientation.z,
                         odom_optflow.pose.pose.orientation.w);
    quat = quat.inverse();

    tf.transform.rotation.x = quat.x();
    tf.transform.rotation.y = quat.y();
    tf.transform.rotation.z = quat.z();
    tf.transform.rotation.w = quat.w();

    try {
      tf_broadcaster->sendTransform(tf);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing TF: %s - %s.", tf.child_frame_id.c_str(), tf.header.frame_id.c_str());
    }
  }

}  // namespace tomasuv_tf_publisher

//}

// --------------------------------------------------------------
// |                       other routines                       |
// --------------------------------------------------------------

}  // namespace tomasuv_tf_publisher

}  // namespace mrs_general

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_general::tomasuv_tf_publisher::TomasuvTfPublisher, nodelet::Nodelet)
