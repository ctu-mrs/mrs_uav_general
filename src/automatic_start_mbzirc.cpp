/* includes //{ */

#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mutex>

#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/mutex.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <mrs_msgs/SetInt.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/MpcTrackerDiagnostics.h>

//}

/* defines //{ */

#define STRING_EQUAL 0

//}

namespace mrs_general
{

namespace automatic_start_mbzirc
{

/* class AutomaticStartMbzirc //{ */

// state machine
typedef enum
{

  IDLE_STATE,
  TAKEOFF_STATE,
  FINISHED_STATE

} LandingStates_t;

const char* state_names[3] = {

    "IDLING", "TAKING OFF", "FINISHED"};

class AutomaticStartMbzirc : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;
  bool            simulation_     = false;

private:
  double _safety_timeout_;

private:
  ros::ServiceClient service_client_motors_;
  ros::ServiceClient service_client_takeoff_;
  ros::ServiceClient service_client_start_;

private:
  ros::Subscriber subscriber_mavros_state_;
  ros::Subscriber subscriber_rc_;
  ros::Subscriber subscriber_control_manager_diagnostics_;

private:
  ros::Timer main_timer_;
  void       mainTimer(const ros::TimerEvent& event);
  double     main_timer_rate_;

private:
  void              callbackRC(const mavros_msgs::RCInConstPtr& msg);
  mavros_msgs::RCIn rc_channels_;
  std::mutex        mutex_rc_channels_;
  bool              got_rc_channels_ = false;

private:
  void       callbackMavrosState(const mavros_msgs::StateConstPtr& msg);
  bool       got_mavros_state_ = false;
  std::mutex mutex_mavros_state_;

  ros::Time armed_time_;
  bool      armed_ = false;

  ros::Time offboard_time_;
  bool      offboard_ = false;

private:
  void                                callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg);
  std::mutex                          mutex_control_manager_diagnostics_;
  mrs_msgs::ControlManagerDiagnostics control_manager_diagnostics_;
  bool                                got_control_manager_diagnostics_ = false;

private:
  bool takeoff();
  bool setMotors(const bool value);
  bool start(const int value);

private:
  uint current_state = IDLE_STATE;
  void changeState(LandingStates_t new_state);
};

//}

/* onInit() //{ */

void AutomaticStartMbzirc::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  armed_      = false;
  armed_time_ = ros::Time(0);

  offboard_      = false;
  offboard_time_ = ros::Time(0);

  mrs_lib::ParamLoader param_loader(nh_, "AutomaticStartMbzirc");

  param_loader.load_param("safety_timeout", _safety_timeout_);
  param_loader.load_param("main_timer_rate", main_timer_rate_);
  param_loader.load_param("simulation", simulation_);

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[MavrosInterface]: Could not load all parameters!");
    ros::shutdown();
  }

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_mavros_state_ = nh_.subscribe("mavros_state_in", 1, &AutomaticStartMbzirc::callbackMavrosState, this, ros::TransportHints().tcpNoDelay());
  subscriber_control_manager_diagnostics_ =
      nh_.subscribe("control_manager_diagnostics_in", 1, &AutomaticStartMbzirc::callbackControlManagerDiagnostics, this, ros::TransportHints().tcpNoDelay());
  subscriber_rc_ = nh_.subscribe("rc_in", 1, &AutomaticStartMbzirc::callbackRC, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                       service clients                      |
  // --------------------------------------------------------------

  service_client_takeoff_ = nh_.serviceClient<std_srvs::Trigger>("takeoff_out");
  service_client_motors_  = nh_.serviceClient<std_srvs::SetBool>("motors_out");
  service_client_start_   = nh_.serviceClient<mrs_msgs::SetInt>("start_out");

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  main_timer_ = nh_.createTimer(ros::Rate(main_timer_rate_), &AutomaticStartMbzirc::mainTimer, this);

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackMavrosState() //{ */

void AutomaticStartMbzirc::callbackMavrosState(const mavros_msgs::StateConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[AutomaticStartMbzirc]: getting mavros state");

  std::scoped_lock lock(mutex_mavros_state_);

  // check armed_ state
  if (armed_ == false) {

    // if armed_ state changed to true, please "start the clock"
    if (msg->armed > 0) {

      armed_      = true;
      armed_time_ = ros::Time::now();
    }

    // if we were armed_ previously
  } else if (armed_ == true) {

    // and we are not really now
    if (msg->armed == 0) {

      armed_ = false;
    }
  }

  // check offboard_ state
  if (offboard_ == false) {

    // if offboard_ state changed to true, please "start the clock"
    if (msg->mode == "OFFBOARD") {

      offboard_      = true;
      offboard_time_ = ros::Time::now();
    }

    // if we were in offboard_ previously
  } else if (offboard_ == true) {

    // and we are not really now
    if (msg->mode != "OFFBOARD") {

      offboard_ = false;
    }
  }

  got_mavros_state_ = true;
}

//}

/* callbackControlManagerDiagnostics() //{ */

void AutomaticStartMbzirc::callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[AutomaticStartMbzirc]: getting control manager diagnostics");

  {
    std::scoped_lock lock(mutex_control_manager_diagnostics_);

    control_manager_diagnostics_ = *msg;

    got_control_manager_diagnostics_ = true;
  }
}

//}

/* //{ callbackRC() */

void AutomaticStartMbzirc::callbackRC(const mavros_msgs::RCInConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[ControlManager]: getting RC channels");

  {
    std::scoped_lock lock(mutex_rc_channels_);

    rc_channels_ = *msg;

    got_rc_channels_ = true;
  }
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* mainTimer() //{ */

void AutomaticStartMbzirc::mainTimer([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  if (!got_control_manager_diagnostics_ || !got_mavros_state_ || !(got_rc_channels_ || simulation_)) {
    ROS_WARN_THROTTLE(5.0, "[AutomaticStartMbzirc]: waiting for data: ControManager=%s, Mavros=%s, RC=%s", got_control_manager_diagnostics_ ? "true" : "FALSE",
                      got_mavros_state_ ? "true" : "FALSE", got_rc_channels_ ? "true" : "FALSE");
    return;
  }

  auto [armed, offboard, armed_time, offboard_time] = mrs_lib::get_mutexed(mutex_mavros_state_, armed_, offboard_, armed_time_, offboard_time_);
  auto control_manager_diagnostics                  = mrs_lib::get_mutexed(mutex_control_manager_diagnostics_, control_manager_diagnostics_);
  auto rc_channels                                  = mrs_lib::get_mutexed(mutex_rc_channels_, rc_channels_);

  bool motors = control_manager_diagnostics.motors;

  switch (current_state) {

    case IDLE_STATE: {

      // turn on motors
      if (!motors) {
        setMotors(true);
      }

      // when armed and in offboard, takeoff
      if (armed && offboard && motors) {

        double armed_time_diff    = (ros::Time::now() - armed_time).toSec();
        double offboard_time_diff = (ros::Time::now() - offboard_time).toSec();

        if ((armed_time_diff > _safety_timeout_) && (offboard_time_diff > _safety_timeout_)) {

          changeState(TAKEOFF_STATE);

        } else {

          double min = (armed_time_diff < offboard_time_diff) ? armed_time_diff : offboard_time_diff;

          ROS_WARN_THROTTLE(1.0, "Starting in %1.0f", (_safety_timeout_ - min));
        }
      }

      break;
    }

    case TAKEOFF_STATE: {

      std::scoped_lock lock(mutex_control_manager_diagnostics_);

      // if takeoff finished
      if (control_manager_diagnostics.tracker_status.tracker == "MpcTracker" && control_manager_diagnostics.tracker_status.callbacks_enabled) {

        ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: takeoff finished");

        changeState(FINISHED_STATE);
      }

      break;
    }

    case FINISHED_STATE: {

      ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: we are done here");

      ros::Duration(3.0).sleep();
      ros::shutdown();

      break;
    }
  }

}  // namespace automatic_start_mbzirc

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* changeState() //{ */

void AutomaticStartMbzirc::changeState(LandingStates_t new_state) {

  ROS_WARN_THROTTLE(1.0, "[AutomaticStartMbzirc]: switching states %s -> %s", state_names[current_state], state_names[new_state]);

  switch (new_state) {

    case IDLE_STATE: {

      break;
    }

    case TAKEOFF_STATE: {

      bool res = takeoff();

      if (!res) {
        return;
      }

      break;
    }

    case FINISHED_STATE: {

      bool res = start(0);

      if (!res) {
        return;
      }

      break;
    }

    break;
  }

  current_state = new_state;
}

//}

/* takeoff() //{ */

bool AutomaticStartMbzirc::takeoff() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: taking off");

  std_srvs::Trigger srv;

  bool res = service_client_takeoff_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: taking off failed: %s", srv.response.message.c_str());
    }

  } else if (!srv.response.success) {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for taking off failed");
  }

  return false;
}

//}

/* motors() //{ */

bool AutomaticStartMbzirc::setMotors(const bool value) {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: setting motors %s", value ? "ON" : "OFF");

  std_srvs::SetBool srv;
  srv.request.data = value;

  bool res = service_client_motors_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: setting motors failed: %s", srv.response.message.c_str());
    }

  } else if (!srv.response.success) {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for setting motors failed");
  }

  return false;
}

//}

/* start() //{ */

bool AutomaticStartMbzirc::start(const int value) {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: starting action");

  mrs_msgs::SetInt srv;
  srv.request.value = value;

  bool res = service_client_start_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: starting action failed failed: %s", srv.response.message.c_str());
    }

  } else if (!srv.response.success) {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for starting action failed");
  }

  return false;
}

//}

}  // namespace automatic_start_mbzirc

}  // namespace mrs_general

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_general::automatic_start_mbzirc::AutomaticStartMbzirc, nodelet::Nodelet)
