/* includes //{ */

#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mutex>

#include <mavros_msgs/State.h>

#include <nav_msgs/Odometry.h>

#include <std_srvs/Trigger.h>
#include <mrs_msgs/String.h>

#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/MpcTrackerDiagnostics.h>

#include <mrs_lib/ParamLoader.h>

//}

/* defines //{ */

#define STRING_EQUAL 0

//}

namespace mrs_general
{

namespace automatic_start_darpa
{

/* class AutomaticStartDarpa //{ */

// state machine
typedef enum
{

  IDLE_STATE,
  TAKEOFF_STATE,
  FLYING_IN_STATE,
  FLYING_OUT_STATE,
  LANDING_STATE,
  FINISHED_STATE

} LandingStates_t;

const char* state_names[7] = {

    "IDLING", "TAKING OFF", "FLYING IN", "FLYING OUT", "LANDING", "FINISHED"};

class AutomaticStartDarpa : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized   = false;
  bool            april_tag_killed = false;
  std::string     scripts_path_;
  double          shutdown_timeout_;

private:
  double safety_timeout_;

private:
  ros::ServiceClient service_client_takeoff;
  ros::ServiceClient service_client_goto;
  ros::ServiceClient service_client_return;
  ros::ServiceClient service_client_land;
  ros::ServiceClient service_client_estimator;
  ros::ServiceClient service_client_hdg_estimator;

private:
  ros::ServiceServer service_server_shutdown;

private:
  ros::Subscriber subscriber_mavros_state;
  ros::Subscriber subscriber_control_manager_diagnostics;
  ros::Subscriber subscriber_odometry;

private:
  ros::Timer main_timer;
  void       mainTimer(const ros::TimerEvent& event);
  double     main_timer_rate_;

  double    return_time_;
  ros::Time start_time;
  double    start_x, start_y;

private:
  void       callbackMavrosState(const mavros_msgs::StateConstPtr& msg);
  std::mutex mutex_mavros_state;

  ros::Time armed_time;
  bool      armed = false;

  ros::Time offboard_time;
  bool      offboard = false;

private:
  void                                callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg);
  std::mutex                          mutex_control_manager_diagnostics;
  mrs_msgs::ControlManagerDiagnostics control_manager_diagnostics;
  bool                                got_control_manager_diagnostics = false;

private:
  void               callbackOdometry(const nav_msgs::OdometryConstPtr& msg);
  std::mutex         mutex_odometry;
  nav_msgs::Odometry odometry;
  bool               got_odometry = false;

private:
  uint current_state = IDLE_STATE;
  void changeState(LandingStates_t new_state);

private:
  double goto_x_, goto_y_, goto_z_, goto_yaw_;

private:
  bool       callbackShutdown([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::Timer shutdown_timer;
  void       shutdownTimer(const ros::TimerEvent& event);
  ros::Time  shutdown_time;

  double dist2(const double x1, const double y1, const double x2, const double y2);
};

//}

/* inInit() //{ */

void AutomaticStartDarpa::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  armed      = false;
  armed_time = ros::Time(0);

  offboard      = false;
  offboard_time = ros::Time(0);

  mrs_lib::ParamLoader param_loader(nh_, "AutomaticStartDarpa");

  param_loader.load_param("safety_timeout", safety_timeout_);
  param_loader.load_param("main_timer_rate", main_timer_rate_);

  param_loader.load_param("goto/x", goto_x_);
  param_loader.load_param("goto/y", goto_y_);
  param_loader.load_param("goto/z", goto_z_);
  param_loader.load_param("goto/yaw", goto_yaw_);

  param_loader.load_param("scripts_path", scripts_path_);
  param_loader.load_param("shutdown_timeout", shutdown_timeout_);
  param_loader.load_param("return_timer", return_time_);

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_mavros_state = nh_.subscribe("mavros_state_in", 1, &AutomaticStartDarpa::callbackMavrosState, this, ros::TransportHints().tcpNoDelay());
  subscriber_control_manager_diagnostics =
      nh_.subscribe("control_manager_diagnostics_in", 1, &AutomaticStartDarpa::callbackControlManagerDiagnostics, this, ros::TransportHints().tcpNoDelay());
  subscriber_odometry = nh_.subscribe("odometry_in", 1, &AutomaticStartDarpa::callbackOdometry, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                       service clients                      |
  // --------------------------------------------------------------

  service_client_takeoff       = nh_.serviceClient<std_srvs::Trigger>("takeoff_out");
  service_client_goto          = nh_.serviceClient<mrs_msgs::Vec4>("goto_out");
  service_client_return        = nh_.serviceClient<std_srvs::Trigger>("return_out");
  service_client_land          = nh_.serviceClient<std_srvs::Trigger>("land_out");
  service_client_estimator     = nh_.serviceClient<mrs_msgs::String>("change_estimator_out");
  service_client_hdg_estimator = nh_.serviceClient<mrs_msgs::String>("change_hdg_estimator_out");

  // --------------------------------------------------------------
  // |                       service servers                      |
  // --------------------------------------------------------------

  service_server_shutdown = nh_.advertiseService("shutdown_in", &AutomaticStartDarpa::callbackShutdown, this);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  main_timer     = nh_.createTimer(ros::Rate(main_timer_rate_), &AutomaticStartDarpa::mainTimer, this);
  shutdown_timer = nh_.createTimer(ros::Rate(1.0), &AutomaticStartDarpa::shutdownTimer, this, false, false);

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[MavrosInterface]: Could not load all parameters!");
    ros::shutdown();
  }

  is_initialized = true;

  ROS_INFO("[AutomaticStartDarpa]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackMavrosState() //{ */

void AutomaticStartDarpa::callbackMavrosState(const mavros_msgs::StateConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  std::scoped_lock lock(mutex_mavros_state);

  ROS_INFO_ONCE("[AutomaticStartDarpa]: getting mavros state");

  // check armed state
  if (armed == false) {

    // if armed state changed to true, please "start the clock"
    if (msg->armed > 0) {

      armed      = true;
      armed_time = ros::Time::now();
    }

    // if we were armed previously
  } else if (armed == true) {

    // and we are not really now
    if (msg->armed == 0) {

      armed = false;
    }
  }

  // check offboard state
  if (offboard == false) {

    // if offboard state changed to true, please "start the clock"
    if (msg->mode.compare(std::string("OFFBOARD")) == 0) {

      offboard      = true;
      offboard_time = ros::Time::now();
    }

    // if we were in offboard previously
  } else if (offboard == true) {

    // and we are not really now
    if (msg->mode.compare(std::string("OFFBOARD")) != 0) {

      offboard = false;
    }
  }
}

//}

/* callbackControlManagerDiagnostics() //{ */

void AutomaticStartDarpa::callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  std::scoped_lock lock(mutex_control_manager_diagnostics);

  ROS_INFO_ONCE("[AutomaticStartDarpa]: getting control manager diagnostics");

  got_control_manager_diagnostics = true;

  control_manager_diagnostics = *msg;
}

//}

/* callbackOdometry() //{ */

void AutomaticStartDarpa::callbackOdometry(const nav_msgs::OdometryConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  std::scoped_lock lock(mutex_odometry);

  ROS_INFO_ONCE("[AutomaticStartDarpa]: getting odometry");

  got_odometry = true;

  odometry = *msg;
}

//}

/* callbackShutdown() //{ */

bool AutomaticStartDarpa::callbackShutdown([[maybe_unused]] std_srvs::Trigger::Request& req, [[maybe_unused]] std_srvs::Trigger::Response& res) {

  if (!is_initialized)
    return false;

  shutdown_time = ros::Time::now();
  shutdown_timer.start();

  res.success = true;
  res.message = "shutting down";

  return true;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* changeState() //{ */

void AutomaticStartDarpa::changeState(LandingStates_t new_state) {

  switch (new_state) {

    case IDLE_STATE: {

      break;
    }

    case TAKEOFF_STATE: {

      std_srvs::Trigger trigger_out;

      if (!service_client_takeoff.call(trigger_out)) {

        ROS_ERROR("[AutomaticStartDarpa]: service call for takeoff failed");

      } else if (!trigger_out.response.success) {

        ROS_ERROR("[AutomaticStartDarpa]: service fall for takeoff returned false: %s", trigger_out.response.message.c_str());

      } else {

        ROS_INFO("[AutomaticStartDarpa]: service call for takeoff suceeeded");
      }

      break;
    }

    case FLYING_IN_STATE: {

      mrs_msgs::Vec4 goto_out;
      goto_out.request.goal[0] = goto_x_;
      goto_out.request.goal[1] = goto_y_;
      goto_out.request.goal[2] = goto_z_;
      goto_out.request.goal[3] = goto_yaw_;

      {
        std::scoped_lock lock(mutex_odometry);

        start_x    = odometry.pose.pose.position.x;
        start_y    = odometry.pose.pose.position.y;
        start_time = ros::Time::now();
      }

      ROS_INFO("[AutomaticStartDarpa]: calling goto");

      if (!service_client_goto.call(goto_out)) {

        ROS_ERROR("[AutomaticStartDarpa]: service call for goto failed");

      } else if (!goto_out.response.success) {

        ROS_ERROR("[AutomaticStartDarpa]: service fall for goto returned false: %s", goto_out.response.message.c_str());

      } else {

        ROS_INFO("[AutomaticStartDarpa]: service call for goto suceeeded");
      }

      break;
    }

    case FLYING_OUT_STATE: {

      std_srvs::Trigger trigger_out;

      if (!service_client_return.call(trigger_out)) {

        ROS_ERROR("[AutomaticStartDarpa]: service call for returning failed");

      } else if (!trigger_out.response.success) {

        ROS_ERROR("[AutomaticStartDarpa]: service fall for returning returned false: %s", trigger_out.response.message.c_str());

      } else {

        ROS_INFO("[AutomaticStartDarpa]: service call for returning suceeeded");
      }

      break;
    }

    case LANDING_STATE: {

      std_srvs::Trigger trigger_out;

      if (!service_client_land.call(trigger_out)) {

        ROS_ERROR("[AutomaticStartDarpa]: service call for landing failed");

      } else if (!trigger_out.response.success) {

        ROS_ERROR("[AutomaticStartDarpa]: service fall for landing returned false: %s", trigger_out.response.message.c_str());

      } else {

        ROS_INFO("[AutomaticStartDarpa]: service call for landing suceeeded");
      }

      shutdown_timer.stop();

      break;
    }

    case FINISHED_STATE: {

      break;
    }

    break;
  }

  ROS_WARN("[AutomaticStartDarpa]: switching states %s -> %s", state_names[current_state], state_names[new_state]);

  current_state = new_state;
}

//}

/* mainTimer() //{ */

void AutomaticStartDarpa::mainTimer([[maybe_unused]] const ros::TimerEvent& event) {

  switch (current_state) {

    case IDLE_STATE: {

      std::scoped_lock lock(mutex_mavros_state);

      if (armed && offboard) {

        double armed_time_diff    = (ros::Time::now() - armed_time).toSec();
        double offboard_time_diff = (ros::Time::now() - offboard_time).toSec();

        if ((armed_time_diff > safety_timeout_) && (offboard_time_diff > safety_timeout_)) {

          ROS_ERROR("STARTING THE STATE MACHINE!!!!");

          changeState(TAKEOFF_STATE);

        } else {

          double min = (armed_time_diff < offboard_time_diff) ? armed_time_diff : offboard_time_diff;

          ROS_WARN_THROTTLE(1, "Starting in %1.0f", (safety_timeout_ - min));
        }
      }

      break;
    }

    case TAKEOFF_STATE: {

      std::scoped_lock lock(mutex_control_manager_diagnostics);

      // if takeoff finished
      if (control_manager_diagnostics.tracker_status.tracker.compare("MpcTracker") && control_manager_diagnostics.tracker_status.callbacks_enabled) {

        ROS_INFO("[AutomaticStartDarpa]: takeoff finished");

        mrs_msgs::String string_out;

        ROS_INFO("[AutomaticStartDarpa]: switching to HECTOR estimators");

        string_out.request.value = "HECTOR";

        if (!service_client_hdg_estimator.call(string_out)) {

          ROS_ERROR("[AutomaticStartDarpa]: service call for HDG estimator switch failed");

        } else if (!string_out.response.success) {

          ROS_ERROR("[AutomaticStartDarpa]: service call for HDG estimator switch failed: %s", string_out.response.message.c_str());
        } else {

          ROS_INFO("[AutomaticStartDarpa]: switching of HDG estimator succeeeeded");
        }

        if (!service_client_estimator.call(string_out)) {

          ROS_ERROR("[AutomaticStartDarpa]: service for lateral estimator switch failed");

        } else if (!string_out.response.success) {

          ROS_ERROR("[AutomaticStartDarpa]: service call for lateral estimator switch failed: %s", string_out.response.message.c_str());

        } else {

          ROS_INFO("[AutomaticStartDarpa]: switching of lateral estimator succeeeeded");
        }

        ros::Duration(1.0).sleep();

        changeState(FLYING_IN_STATE);
      }

      break;
    }

    case FLYING_IN_STATE: {

      if (dist2(odometry.pose.pose.position.x, odometry.pose.pose.position.y, start_x, start_y) > 5.0 && !april_tag_killed) {
        ROS_INFO("[AutomaticStartDarpa]: calling for apriltag node kill");
        [[maybe_unused]] int res = system((scripts_path_ + std::string("/kill_apriltag.sh")).c_str());
        april_tag_killed         = true;
      }

      if ((ros::Time::now() - start_time).toSec() > return_time_) {

        ROS_INFO("[AutomaticStartDarpa]: flying over %.2f s, returning home", return_time_);

        changeState(FLYING_OUT_STATE);
      }

      break;
    }

    case FLYING_OUT_STATE: {

      std::scoped_lock lock(mutex_odometry);

      if (dist2(odometry.pose.pose.position.x, odometry.pose.pose.position.y, start_x, start_y) < 1.0) {

        ROS_INFO("[AutomaticStartDarpa]: reached the starting point, landing");

        changeState(LANDING_STATE);
      }

      break;
    }
  }

}  // namespace automatic_start_darpa

//}

/* shutdownTimer() //{ */

void AutomaticStartDarpa::shutdownTimer([[maybe_unused]] const ros::TimerEvent& event) {

  double time = (ros::Time::now() - shutdown_time).toSec();

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartDarpa]: shutting down in %d s", int(shutdown_timeout_ - time));

  if (time > shutdown_timeout_) {

    ROS_INFO("[AutomaticStartDarpa]: calling for shutdown");
    [[maybe_unused]] int res = system((scripts_path_ + std::string("/shutdown.sh")).c_str());
  }
}

//}

// --------------------------------------------------------------
// |                       other routines                       |
// --------------------------------------------------------------

/* dist2() //{ */

double AutomaticStartDarpa::dist2(const double x1, const double y1, const double x2, const double y2) {

  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

//}

}  // namespace automatic_start_darpa

}  // namespace mrs_general

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_general::automatic_start_darpa::AutomaticStartDarpa, nodelet::Nodelet)
