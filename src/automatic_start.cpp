#define VERSION "0.0.5.0"

/* includes //{ */

#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mutex>

#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/mutex.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/MpcTrackerDiagnostics.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/ValidateReference.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/CameraInfo.h>

//}

namespace mrs_general
{

namespace automatic_start
{

/* class AutomaticStart //{ */

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

class AutomaticStart : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;
  bool            _simulation_    = false;
  std::string     _version_;

  // | --------------------- service clients -------------------- |

  ros::ServiceClient service_client_motors_;
  ros::ServiceClient service_client_arm_;
  ros::ServiceClient service_client_takeoff_;
  ros::ServiceClient service_client_land_home_;
  ros::ServiceClient service_client_land_;
  ros::ServiceClient service_client_eland_;
  ros::ServiceClient service_client_validate_reference_;
  ros::ServiceClient service_client_start_;
  ros::ServiceClient service_client_stop_;

  // | ----------------------- subscribers ---------------------- |

  ros::Subscriber subscriber_mavros_state_;
  ros::Subscriber subscriber_control_manager_diagnostics_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent& event);
  double     _main_timer_rate_;

  // | ---------------------- mavros state ---------------------- |

  void       callbackMavrosState(const mavros_msgs::StateConstPtr& msg);
  bool       got_mavros_state_ = false;
  std::mutex mutex_mavros_state_;

  // | ----------------- arm and offboard check ----------------- |

  ros::Time armed_time_;
  bool      armed_ = false;

  ros::Time offboard_time_;
  bool      offboard_ = false;

  // | --------------- control manager diagnostics -------------- |

  void                                callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg);
  std::mutex                          mutex_control_manager_diagnostics_;
  mrs_msgs::ControlManagerDiagnostics control_manager_diagnostics_;
  bool                                got_control_manager_diagnostics_ = false;

  // | ------------------------ routines ------------------------ |

  bool takeoff();

  bool landImpl();
  bool elandImpl();
  bool landHomeImpl();
  bool land();
  void validateReference();

  bool setMotors(const bool value);
  bool disarm();
  bool start(void);
  bool stop();

  // | ---------------------- other params ---------------------- |

  double      _action_duration_;
  double      _pre_takeoff_sleep_;
  bool        _handle_landing_ = false;
  bool        _handle_takeoff_ = false;
  std::string _land_mode_;
  double      _safety_timeout_;

  // | ---------------------- start service --------------------- |

  int       _start_n_attempts_;
  int       call_attempt_counter_ = 0;
  ros::Time start_time_;

  // | ---------------------- state machine --------------------- |

  uint current_state = STATE_IDLE;
  void changeState(LandingStates_t new_state);
};

//}

/* onInit() //{ */

void AutomaticStart::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  armed_      = false;
  armed_time_ = ros::Time(0);

  offboard_      = false;
  offboard_time_ = ros::Time(0);

  mrs_lib::ParamLoader param_loader(nh_, "AutomaticStart");

  param_loader.load_param("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[AutomaticStart]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.load_param("safety_timeout", _safety_timeout_);
  param_loader.load_param("main_timer_rate", _main_timer_rate_);
  param_loader.load_param("simulation", _simulation_);
  param_loader.load_param("call_n_attempts", _start_n_attempts_);

  param_loader.load_param("land_mode", _land_mode_);
  param_loader.load_param("handle_landing", _handle_landing_);
  param_loader.load_param("handle_takeoff", _handle_takeoff_);
  param_loader.load_param("action_duration", _action_duration_);
  param_loader.load_param("pre_takeoff_sleep", _pre_takeoff_sleep_);

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[AutomaticStart]: Could not load all parameters!");
    ros::shutdown();
  }

  // recaltulate the acion duration to seconds
  _action_duration_ *= 60;

  if (!(_land_mode_ == "land_home" || _land_mode_ == "land" || _land_mode_ == "eland")) {

    ROS_ERROR("[AutomaticStart]: land_mode ('%s') was specified wrongly, will eland by default!!!", _land_mode_.c_str());
  }

  // | ----------------------- subscribers ---------------------- |

  subscriber_mavros_state_ = nh_.subscribe("mavros_state_in", 1, &AutomaticStart::callbackMavrosState, this, ros::TransportHints().tcpNoDelay());
  subscriber_control_manager_diagnostics_ =
      nh_.subscribe("control_manager_diagnostics_in", 1, &AutomaticStart::callbackControlManagerDiagnostics, this, ros::TransportHints().tcpNoDelay());

  // | --------------------- service clients -------------------- |

  service_client_takeoff_   = nh_.serviceClient<std_srvs::Trigger>("takeoff_out");
  service_client_land_home_ = nh_.serviceClient<std_srvs::Trigger>("land_home_out");
  service_client_land_      = nh_.serviceClient<std_srvs::Trigger>("land_out");
  service_client_eland_     = nh_.serviceClient<std_srvs::Trigger>("eland_out");
  service_client_motors_    = nh_.serviceClient<std_srvs::SetBool>("motors_out");
  service_client_arm_       = nh_.serviceClient<mavros_msgs::CommandBool>("arm_out");

  service_client_validate_reference_ = nh_.serviceClient<mrs_msgs::ValidateReference>("validate_reference_out");

  service_client_start_ = nh_.serviceClient<std_srvs::Trigger>("start_out");

  service_client_stop_ = nh_.serviceClient<std_srvs::Trigger>("stop_out");

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(_main_timer_rate_), &AutomaticStart::timerMain, this);

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: initialized, version %s", VERSION);
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackMavrosState() //{ */

void AutomaticStart::callbackMavrosState(const mavros_msgs::StateConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[AutomaticStart]: getting mavros state");

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

void AutomaticStart::callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[AutomaticStart]: getting control manager diagnostics");

  {
    std::scoped_lock lock(mutex_control_manager_diagnostics_);

    control_manager_diagnostics_ = *msg;

    got_control_manager_diagnostics_ = true;
  }
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void AutomaticStart::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  if (!got_control_manager_diagnostics_ || !got_mavros_state_) {
    ROS_WARN_THROTTLE(5.0, "[AutomaticStart]: waiting for data: ControManager=%s, Mavros=%s", got_control_manager_diagnostics_ ? "true" : "FALSE",
                      got_mavros_state_ ? "true" : "FALSE");
    return;
  }

  auto [armed, offboard, armed_time, offboard_time] = mrs_lib::get_mutexed(mutex_mavros_state_, armed_, offboard_, armed_time_, offboard_time_);
  auto control_manager_diagnostics                  = mrs_lib::get_mutexed(mutex_control_manager_diagnostics_, control_manager_diagnostics_);

  bool   motors           = control_manager_diagnostics.motors;
  double time_from_arming = (ros::Time::now() - armed_time).toSec();

  switch (current_state) {

    case STATE_IDLE: {

      validateReference();

      if (armed && !motors) {

        double res = setMotors(true);

        if (!res) {
          ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: could not set motors ON");
        }

        if (time_from_arming > 1.5) {

          ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: could not set motors ON for 1.5 secs, disarming");
          disarm();
        }
      }

      // when armed and in offboard, takeoff
      if (armed && offboard && motors) {

        double armed_time_diff    = (ros::Time::now() - armed_time).toSec();
        double offboard_time_diff = (ros::Time::now() - offboard_time).toSec();

        if ((armed_time_diff > _safety_timeout_) && (offboard_time_diff > _safety_timeout_)) {

          if (_handle_takeoff_) {
            changeState(STATE_TAKEOFF);
          } else {
            changeState(STATE_IN_ACTION);
          }

        } else {

          double min = (armed_time_diff < offboard_time_diff) ? armed_time_diff : offboard_time_diff;

          ROS_WARN_THROTTLE(1.0, "starting in %.0f", (_safety_timeout_ - min));
        }
      }

      break;
    }

    case STATE_TAKEOFF: {

      std::scoped_lock lock(mutex_control_manager_diagnostics_);

      // if takeoff finished
      if (control_manager_diagnostics.active_tracker == "MpcTracker" && !control_manager_diagnostics.tracker_status.moving_reference) {

        ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: takeoff finished");

        changeState(STATE_IN_ACTION);
      }

      break;
    }

    case STATE_IN_ACTION: {

      if (_handle_landing_) {

        double in_action_time = (ros::Time::now() - start_time_).toSec();

        ROS_INFO_THROTTLE(5.0, "[AutomaticStart]: in action for %.0f second out of %.0f", in_action_time, _action_duration_);

        if (in_action_time > _action_duration_) {

          ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: the action duration time has passed, landing");

          changeState(STATE_LAND);
        }

      } else {

        changeState(STATE_FINISHED);
      }

      break;
    }

    case STATE_LAND: {

      if (!armed || !offboard || !motors) {

        ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: the UAV has probably landed");

        changeState(STATE_FINISHED);
      }

      break;
    }

    case STATE_FINISHED: {

      ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: we are done here");

      break;
    }
  }

}  // namespace automatic_start

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* changeState() //{ */

void AutomaticStart::changeState(LandingStates_t new_state) {

  ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: switching states %s -> %s", state_names[current_state], state_names[new_state]);

  switch (new_state) {

    case STATE_IDLE: {

      break;
    }

    case STATE_TAKEOFF: {

      if (_pre_takeoff_sleep_ > 1.0) {
        ROS_INFO("[AutomaticStart]: sleeping for %.2f secs before takeoff", _pre_takeoff_sleep_);
        ros::Duration(_pre_takeoff_sleep_).sleep();
      }

      bool res = takeoff();

      if (!res) {
        return;
      }

      break;
    }

    case STATE_IN_ACTION: {

      bool res = start();

      if (!res) {

        if (++call_attempt_counter_ < _start_n_attempts_) {

          ROS_WARN("[AutomaticStart]: failed to call start, attempting again");
          return;

        } else {

          ROS_ERROR("[AutomaticStart]: failed to call start for the %dth time, giving up", call_attempt_counter_);
        }
      }

      call_attempt_counter_ = 0;

      start_time_ = ros::Time::now();

      break;
    }

    case STATE_LAND: {

      {
        bool res = stop();

        if (++call_attempt_counter_ < _start_n_attempts_) {

          ROS_WARN("[AutomaticStart]: failed to call stop, attempting again");

          if (!res) {
            return;
          }

        } else {

          ROS_ERROR("[AutomaticStart]: failed to call stop for the %dth time, giving up", call_attempt_counter_);
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

bool AutomaticStart::takeoff() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: taking off");

  std_srvs::Trigger srv;

  bool res = service_client_takeoff_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: taking off failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for taking off failed");
  }

  return false;
}

//}

/* landHomeImpl() //{ */

bool AutomaticStart::landHomeImpl() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: landing home");

  std_srvs::Trigger srv;

  bool res = service_client_land_home_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: landing home failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for landing home failed");
  }

  return false;
}

//}

/* landImpl() //{ */

bool AutomaticStart::landImpl() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: landing");

  std_srvs::Trigger srv;

  bool res = service_client_land_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: landing failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for landing failed");
  }

  return false;
}

//}

/* elandImpl() //{ */

bool AutomaticStart::elandImpl() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: elanding");

  std_srvs::Trigger srv;

  bool res = service_client_eland_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: elanding failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for elanding failed");
  }

  return false;
}

//}

/* land() //{ */

bool AutomaticStart::land() {

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

/* validateReference() //{ */

void AutomaticStart::validateReference() {

  mrs_msgs::ValidateReference srv_out;

  srv_out.request.reference.header.frame_id      = "fcu";
  srv_out.request.reference.reference.position.z = 3.0;

  bool res = service_client_validate_reference_.call(srv_out);

  if (res) {

    if (srv_out.response.success) {

      ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: current pos is inside of the safety area");

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: the current pos is outside of the safety area!");
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: current pos could not be validated");
  }
}

//}

/* motors() //{ */

bool AutomaticStart::setMotors(const bool value) {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: setting motors %s", value ? "ON" : "OFF");

  std_srvs::SetBool srv;
  srv.request.data = value;

  bool res = service_client_motors_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: setting motors failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for setting motors failed");
  }

  return false;
}

//}

/* disarm() //{ */

bool AutomaticStart::disarm() {

  if (!got_mavros_state_) {

    ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: cannot not disarm, missing mavros state!");

    return false;
  }

  auto [armed, offboard, armed_time, offboard_time] = mrs_lib::get_mutexed(mutex_mavros_state_, armed_, offboard_, armed_time_, offboard_time_);
  auto control_manager_diagnostics                  = mrs_lib::get_mutexed(mutex_control_manager_diagnostics_, control_manager_diagnostics_);

  if (offboard) {

    ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: cannot not disarm, not in offboard mode!");

    return false;
  }

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: disarming");

  mavros_msgs::CommandBool srv;
  srv.request.value = 0;

  bool res = service_client_arm_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: disarming failed");
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for disarming failed");
  }

  return false;
}

//}

/* start() //{ */

bool AutomaticStart::start(void) {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: starting action");

  std_srvs::Trigger srv;

  bool res = service_client_start_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: starting action failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for starting action failed");
  }

  return false;
}

//}

/* stop() //{ */

bool AutomaticStart::stop() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: stopping action");

  std_srvs::Trigger srv;

  bool res = service_client_stop_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: stopping action failed failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for stopping action failed");
  }

  return false;
}

//}

}  // namespace automatic_start

}  // namespace mrs_general

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_general::automatic_start::AutomaticStart, nodelet::Nodelet)
