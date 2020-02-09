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
#include <mavros_msgs/CommandBool.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <mrs_msgs/SetInt.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/MpcTrackerDiagnostics.h>

//}

/* defines //{ */

#define PWM_LOW_THIRD 1250
#define PWM_HIGH_THIRD 1750

//}

namespace mrs_general
{

namespace automatic_start_mbzirc
{

/* class AutomaticStartMbzirc //{ */

// state machine
typedef enum
{

  STATE_IDLE,
  STATE_TAKEOFF,
  STATE_IN_ACTION,
  STATE_LAND,
  STATE_FINISHED

} LandingStates_t;

const char* state_names[5] = {

    "IDLING", "TAKING OFF", "IN ACTION", "LANDING", "FINISHED"};

class AutomaticStartMbzirc : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;
  bool            _simulation_    = false;
  std::string     _challenge_     = "";

private:
  double _safety_timeout_;

private:
  ros::ServiceClient service_client_motors_;
  ros::ServiceClient service_client_arm_;
  ros::ServiceClient service_client_takeoff_;
  ros::ServiceClient service_client_land_home_;
  ros::ServiceClient service_client_land_;
  ros::ServiceClient service_client_eland_;

  ros::ServiceClient service_client_start_;
  ros::ServiceClient service_client_stop_;

private:
  ros::Subscriber subscriber_mavros_state_;
  ros::Subscriber subscriber_rc_;
  ros::Subscriber subscriber_control_manager_diagnostics_;

private:
  ros::Timer main_timer_;
  void       mainTimer(const ros::TimerEvent& event);
  double     main_timer_rate_;

private:
  void callbackRC(const mavros_msgs::RCInConstPtr& msg);
  bool got_rc_channels_ = false;
  int  _channel_number_;

  int        rc_mode_ = -1;
  std::mutex mutex_rc_mode_;

  int        start_mode_ = -1;
  std::mutex mutex_start_mode_;

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

  bool landImpl();
  bool elandImpl();
  bool landHomeImpl();
  bool land();

  bool setMotors(const bool value);
  bool disarm();
  bool start(const int value);
  bool stop();

private:
  ros::Time start_time_;

  double      _action_duration_;
  bool        _handle_landing_ = false;
  std::string _land_mode_;

private:
  int _start_n_attempts_;
  int call_attempt_counter_ = 0;

private:
  uint current_state = STATE_IDLE;
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
  param_loader.load_param("simulation", _simulation_);
  param_loader.load_param("CHALLENGE", _challenge_);
  param_loader.load_param("channel_number", _channel_number_);
  param_loader.load_param("call_n_attempts", _start_n_attempts_);

  param_loader.load_param("challenges/" + _challenge_ + "/land_mode", _land_mode_);
  param_loader.load_param("challenges/" + _challenge_ + "/handle_landing", _handle_landing_);
  param_loader.load_param("challenges/" + _challenge_ + "/action_duration", _action_duration_);

  // recaltulate the acion duration to seconds
  _action_duration_ *= 60;

  // applies only in simulation
  param_loader.load_param("rc_mode", rc_mode_);

  if (!(_land_mode_ == "land_home" || _land_mode_ == "land" || _land_mode_ == "eland")) {

    ROS_ERROR("[MavrosInterface]: land_mode (\"%s\") was specified wrongly, will eland by default!!!", _land_mode_.c_str());
  }

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

  service_client_takeoff_   = nh_.serviceClient<std_srvs::Trigger>("takeoff_out");
  service_client_land_home_ = nh_.serviceClient<std_srvs::Trigger>("land_home_out");
  service_client_land_      = nh_.serviceClient<std_srvs::Trigger>("land_out");
  service_client_eland_     = nh_.serviceClient<std_srvs::Trigger>("eland_out");
  service_client_motors_    = nh_.serviceClient<std_srvs::SetBool>("motors_out");
  service_client_arm_       = nh_.serviceClient<mavros_msgs::CommandBool>("arm_out");

  if (_challenge_ == "balloons") {

    service_client_start_ = nh_.serviceClient<mrs_msgs::SetInt>("start_out");

  } else if (_challenge_ == "ball") {

    service_client_start_ = nh_.serviceClient<mrs_msgs::SetInt>("start_out");

  } else if (_challenge_ == "fire") {

    service_client_start_ = nh_.serviceClient<mrs_msgs::SetInt>("start_out");

  } else if (_challenge_ == "wall") {

    service_client_start_ = nh_.serviceClient<mrs_msgs::SetInt>("start_out");
  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: FATAL in onInit(): the challenge name is probably wrong");
    ros::shutdown();
  }

  service_client_stop_ = nh_.serviceClient<std_srvs::Trigger>("stop_out");

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

  ROS_INFO_ONCE("[AutomaticStartMbzirc]: getting RC channels");

  std::scoped_lock lock(mutex_rc_mode_);

  if (uint(_channel_number_) >= msg->channels.size()) {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: RC joystick activation channel number (%d) is out of range [0-%d]", uint(_channel_number_),
                       uint(msg->channels.size()));

  } else {

    // detect the switch of a switch on the RC
    if (msg->channels[_channel_number_] < PWM_LOW_THIRD) {

      rc_mode_ = 0;

    } else if ((msg->channels[_channel_number_] >= PWM_LOW_THIRD) && (msg->channels[_channel_number_] <= PWM_HIGH_THIRD)) {

      rc_mode_ = 1;

    } else if (msg->channels[_channel_number_] > PWM_HIGH_THIRD) {

      rc_mode_ = 2;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: the RC channel value is outside of the set of possible values");
    }
  }

  got_rc_channels_ = true;
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

  if (!got_control_manager_diagnostics_ || !got_mavros_state_ || !(got_rc_channels_ || _simulation_)) {
    ROS_WARN_THROTTLE(5.0, "[AutomaticStartMbzirc]: waiting for data: ControManager=%s, Mavros=%s, RC=%s", got_control_manager_diagnostics_ ? "true" : "FALSE",
                      got_mavros_state_ ? "true" : "FALSE", got_rc_channels_ ? "true" : "FALSE");
    return;
  }

  auto [armed, offboard, armed_time, offboard_time] = mrs_lib::get_mutexed(mutex_mavros_state_, armed_, offboard_, armed_time_, offboard_time_);
  auto control_manager_diagnostics                  = mrs_lib::get_mutexed(mutex_control_manager_diagnostics_, control_manager_diagnostics_);
  auto rc_mode                                      = mrs_lib::get_mutexed(mutex_rc_mode_, rc_mode_);

  bool motors = control_manager_diagnostics.motors;

  switch (current_state) {

    case STATE_IDLE: {

      double time_from_arming = (ros::Time::now() - armed_time).toSec();

      double res = setMotors(true);

      if (!res) {

        ROS_WARN_THROTTLE(1.0, "[AutomaticStartMbzirc]: could not set motors ON");
      }

      // disarm if motors are not 
      if (armed && !motors && time_from_arming > 2.0) {

        ROS_WARN_THROTTLE(1.0, "[AutomaticStartMbzirc]: could not set motors ON for 2 secs, disarming");
        disarm();
      }

      // when armed and in offboard, takeoff
      if (armed && offboard && motors) {

        // sae the current rc mode, so it can be later used for start()
        mrs_lib::set_mutexed(mutex_start_mode_, rc_mode, rc_mode_);

        double armed_time_diff    = (ros::Time::now() - armed_time).toSec();
        double offboard_time_diff = (ros::Time::now() - offboard_time).toSec();

        if ((armed_time_diff > _safety_timeout_) && (offboard_time_diff > _safety_timeout_)) {

          changeState(STATE_TAKEOFF);

        } else {

          double min = (armed_time_diff < offboard_time_diff) ? armed_time_diff : offboard_time_diff;

          ROS_WARN_THROTTLE(1.0, "Starting in %1.0f", (_safety_timeout_ - min));
        }
      }

      break;
    }

    case STATE_TAKEOFF: {

      std::scoped_lock lock(mutex_control_manager_diagnostics_);

      // if takeoff finished
      if (control_manager_diagnostics.tracker_status.tracker == "MpcTracker" && control_manager_diagnostics.tracker_status.callbacks_enabled) {

        ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: takeoff finished");

        changeState(STATE_IN_ACTION);
      }

      break;
    }

    case STATE_IN_ACTION: {

      if (_handle_landing_) {

        double in_action_time = (ros::Time::now() - start_time_).toSec();

        ROS_INFO_THROTTLE(5.0, "[AutomaticStartMbzirc]: in action for %.0f second out of %.0f", in_action_time, _action_duration_);

        if (in_action_time > _action_duration_) {

          ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: the action duration time has passed, landing");

          changeState(STATE_LAND);
        }

      } else {

        changeState(STATE_FINISHED);
      }

      break;
    }

    case STATE_LAND: {

      if (!armed || !offboard || !motors) {

        ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: the UAV has probably landed");

        changeState(STATE_FINISHED);
      }

      break;
    }

    case STATE_FINISHED: {

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

  auto start_mode = mrs_lib::get_mutexed(mutex_start_mode_, start_mode_);

  ROS_WARN_THROTTLE(1.0, "[AutomaticStartMbzirc]: switching states %s -> %s", state_names[current_state], state_names[new_state]);

  switch (new_state) {

    case STATE_IDLE: {

      break;
    }

    case STATE_TAKEOFF: {

      bool res = takeoff();

      if (!res) {
        return;
      }

      break;
    }

    case STATE_IN_ACTION: {

      bool res = start(start_mode);

      if (++call_attempt_counter_ < _start_n_attempts_) {

        ROS_WARN("[AutomaticStartMbzirc]: failed to call start, attempting again");

        if (!res) {
          return;
        }
      } else {

        ROS_ERROR("[AutomaticStartMbzirc]: failed to call start for the %dth time, giving up", call_attempt_counter_);
      }

      call_attempt_counter_ = 0;

      start_time_ = ros::Time::now();

      break;
    }

    case STATE_LAND: {

      {
        bool res = stop();

        if (++call_attempt_counter_ < _start_n_attempts_) {

          ROS_WARN("[AutomaticStartMbzirc]: failed to call stop, attempting again");

          if (!res) {
            return;
          }

        } else {

          ROS_ERROR("[AutomaticStartMbzirc]: failed to call stop for the %dth time, giving up", call_attempt_counter_);
        }
      }

      call_attempt_counter_ = 0;

      {
        bool res = land();

        if (!res) {
          return;
        }
      }

      break;
    }

    case STATE_FINISHED: {

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

/* landHomeImpl() //{ */

bool AutomaticStartMbzirc::landHomeImpl() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: landing home");

  std_srvs::Trigger srv;

  bool res = service_client_land_home_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: landing home failed: %s", srv.response.message.c_str());
    }

  } else if (!srv.response.success) {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for landing home failed");
  }

  return false;
}

//}

/* landImpl() //{ */

bool AutomaticStartMbzirc::landImpl() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: landing");

  std_srvs::Trigger srv;

  bool res = service_client_land_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: landing failed: %s", srv.response.message.c_str());
    }

  } else if (!srv.response.success) {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for landing failed");
  }

  return false;
}

//}

/* elandImpl() //{ */

bool AutomaticStartMbzirc::elandImpl() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: elanding");

  std_srvs::Trigger srv;

  bool res = service_client_eland_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: elanding failed: %s", srv.response.message.c_str());
    }

  } else if (!srv.response.success) {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for elanding failed");
  }

  return false;
}

//}

/* land() //{ */

bool AutomaticStartMbzirc::land() {

  bool res;

  if (_land_mode_ == "land") {

    res = landImpl();

  } else if (_land_mode_ == "land_home") {

    res = landHomeImpl();

  } else {

    res = elandImpl();
  }

  return res;
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

/* disarm() //{ */

bool AutomaticStartMbzirc::disarm() {

  if (!got_mavros_state_) {

    ROS_WARN_THROTTLE(1.0, "[AutomaticStartMbzirc]: cannot not disarm, missing mavros state!");

    return false;
  }

  auto [armed, offboard, armed_time, offboard_time] = mrs_lib::get_mutexed(mutex_mavros_state_, armed_, offboard_, armed_time_, offboard_time_);
  auto control_manager_diagnostics                  = mrs_lib::get_mutexed(mutex_control_manager_diagnostics_, control_manager_diagnostics_);

  if (offboard) {

    ROS_WARN_THROTTLE(1.0, "[AutomaticStartMbzirc]: cannot not disarm, not in offboard mode!");

    return false;
  }

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: disarming");

  mavros_msgs::CommandBool srv;
  srv.request.value = 0;

  bool res = service_client_arm_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: disarming failed");
    }

  } else if (!srv.response.success) {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for disarming failed");
  }

  return false;
}

//}

/* start() //{ */

bool AutomaticStartMbzirc::start(const int value) {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: starting action, mode %d", value);

  if (_challenge_ == "balloons") {

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

  } else if (_challenge_ == "ball") {

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

  } else if (_challenge_ == "fire") {

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

  } else if (_challenge_ == "wall") {

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

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: FATAL in start(): the challenge name is wrong");
  }

  return false;
}

//}

/* stop() //{ */

bool AutomaticStartMbzirc::stop() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: stopping action");

  std_srvs::Trigger srv;

  bool res = service_client_stop_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: stopping action failed failed: %s", srv.response.message.c_str());
    }

  } else if (!srv.response.success) {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for stopping action failed");
  }

  return false;
}

//}

}  // namespace automatic_start_mbzirc

}  // namespace mrs_general

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_general::automatic_start_mbzirc::AutomaticStartMbzirc, nodelet::Nodelet)
