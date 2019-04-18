/* includes //{ */

#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mutex>

#include <mavros_msgs/State.h>

#include <std_srvs/Trigger.h>

#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/TrackerStatus.h>
#include <mrs_msgs/TrackerDiagnostics.h>

#include <mrs_lib/ParamLoader.h>

//}

/* defines //{ */

#define STRING_EQUAL 0

//}

namespace mrs_general
{

namespace automatic_start_eagle
{

/* class AutomaticStartEagle //{ */

// state machine
typedef enum
{

  IDLE_STATE,
  TAKEOFF_STATE,
  GOTO_STATE,
  FINISHED_STATE

} LandingStates_t;

const char* state_names[5] = {

    "IDLING", "TAKING OFF", "GOTO", "FINISHED"};

class AutomaticStartEagle : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized = false;
  std::string     scripts_path_;

private:
  double safety_timeout_;
  double action_delay_;

private:
  ros::ServiceClient service_client_takeoff;
  ros::ServiceClient service_client_gofcu;
  ros::ServiceClient service_client_activate;

private:
  ros::Subscriber subscriber_mavros_state;
  ros::Subscriber subscriber_tracker_status;
  ros::Subscriber subscriber_mpc_diagnostics;

private:
  ros::Timer main_timer;
  void       mainTimer(const ros::TimerEvent& event);
  double     main_timer_rate_;

private:
  void       callbackMavrosState(const mavros_msgs::StateConstPtr& msg);
  std::mutex mutex_mavros_state;

  ros::Time armed_time;
  bool      armed = false;

  ros::Time offboard_time;
  bool      offboard = false;

private:
  double goto_x_, goto_y_, goto_z_, goto_yaw_;

private:
  void                         callbackMpcDiagnostics(const mrs_msgs::TrackerDiagnosticsPtr& msg);
  std::mutex                   mutex_mpc_diagnostics;
  mrs_msgs::TrackerDiagnostics mpc_diagnostics;
  bool                         got_mpc_diagnostics;

private:
  void                    callbackTrackerStatus(const mrs_msgs::TrackerStatusConstPtr& msg);
  std::mutex              mutex_tracker_status;
  mrs_msgs::TrackerStatus tracker_status;
  bool                    got_tracker_status = false;

private:
  uint current_state = IDLE_STATE;
};

//}

/* inInit() //{ */

void AutomaticStartEagle::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  armed      = false;
  armed_time = ros::Time(0);

  offboard      = false;
  offboard_time = ros::Time(0);

  mrs_lib::ParamLoader param_loader(nh_, "AutomaticStartEagle");

  param_loader.load_param("safety_timeout", safety_timeout_);
  param_loader.load_param("action_delay", action_delay_);
  param_loader.load_param("main_timer_rate", main_timer_rate_);
  param_loader.load_param("scripts_path", scripts_path_);

  param_loader.load_param("goto/x", goto_x_);
  param_loader.load_param("goto/y", goto_y_);
  param_loader.load_param("goto/z", goto_z_);
  param_loader.load_param("goto/yaw", goto_yaw_);

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_mavros_state    = nh_.subscribe("mavros_state_in", 1, &AutomaticStartEagle::callbackMavrosState, this, ros::TransportHints().tcpNoDelay());
  subscriber_tracker_status  = nh_.subscribe("tracker_status_in", 1, &AutomaticStartEagle::callbackTrackerStatus, this, ros::TransportHints().tcpNoDelay());
  subscriber_mpc_diagnostics = nh_.subscribe("mpc_diagnostics_in", 1, &AutomaticStartEagle::callbackMpcDiagnostics, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                       service clients                      |
  // --------------------------------------------------------------

  service_client_takeoff  = nh_.serviceClient<std_srvs::Trigger>("takeoff_out");
  service_client_gofcu    = nh_.serviceClient<mrs_msgs::Vec4>("gofcu_out");
  service_client_activate = nh_.serviceClient<std_srvs::Trigger>("activate_out");

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  main_timer = nh_.createTimer(ros::Rate(main_timer_rate_), &AutomaticStartEagle::mainTimer, this);

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[MavrosInterface]: Could not load all parameters!");
    ros::shutdown();
  }

  is_initialized = true;

  ROS_INFO("[AutomaticStartEagle]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackMavrosState() //{ */

void AutomaticStartEagle::callbackMavrosState(const mavros_msgs::StateConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  std::scoped_lock lock(mutex_mavros_state);

  ROS_INFO_ONCE("[AutomaticStartEagle]: getting mavros state");

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

/* callbackTrackerStatus() //{ */

void AutomaticStartEagle::callbackTrackerStatus(const mrs_msgs::TrackerStatusConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  std::scoped_lock lock(mutex_tracker_status);

  ROS_INFO_ONCE("[AutomaticStartEagle]: getting tracker status");

  got_tracker_status = true;

  tracker_status = *msg;
}

//}

/* callbackMpcDiagnostics() //{ */

void AutomaticStartEagle::callbackMpcDiagnostics(const mrs_msgs::TrackerDiagnosticsPtr& msg) {

  if (!is_initialized) {
    return;
  }

  std::scoped_lock lock(mutex_mpc_diagnostics);

  ROS_INFO_ONCE("[AutomaticStartEagle]: getting mpc diagnostics");

  got_mpc_diagnostics = true;

  mpc_diagnostics = *msg;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* mainTimer() //{ */

void AutomaticStartEagle::mainTimer([[maybe_unused]] const ros::TimerEvent& event) {

  switch (current_state) {

    case IDLE_STATE: {

      std::scoped_lock lock(mutex_mavros_state);

      if (armed && offboard) {

        double armed_time_diff    = (ros::Time::now() - armed_time).toSec();
        double offboard_time_diff = (ros::Time::now() - offboard_time).toSec();

        if ((armed_time_diff > safety_timeout_) && (offboard_time_diff > safety_timeout_)) {

          ROS_ERROR("STARTING THE STATE MACHINE!!!!");

          std_srvs::Trigger trigger_out;
          service_client_takeoff.call(trigger_out);

          ros::Duration(action_delay_).sleep();

          current_state = TAKEOFF_STATE;

        } else {

          double min = (armed_time_diff < offboard_time_diff) ? armed_time_diff : offboard_time_diff;

          ROS_WARN_THROTTLE(1, "Starting in %1.0f", (safety_timeout_ - min));
        }
      }

      break;
    }

    case TAKEOFF_STATE: {

      std::scoped_lock lock(mutex_tracker_status);

      if (tracker_status.tracker.compare(std::string("mrs_trackers/MpcTracker")) == STRING_EQUAL) {

        ROS_INFO("[AutomaticStartEagle]: takeoff finished");

        mrs_msgs::Vec4 goto_out;
        goto_out.request.goal[0] = goto_x_;
        goto_out.request.goal[1] = goto_y_;
        goto_out.request.goal[2] = goto_z_;
        goto_out.request.goal[3] = goto_yaw_;

        service_client_gofcu.call(goto_out);
        ROS_INFO("[AutomaticStartEagle]: calling goto");

        ros::Duration(action_delay_).sleep();

        current_state = GOTO_STATE;
      }

      break;
    }

    case GOTO_STATE: {

      std::scoped_lock lock(mutex_mpc_diagnostics);

      if (!mpc_diagnostics.tracking_trajectory) {

        ROS_INFO("[AutomaticStartEagle]: reached goal, triggering hunter");

        std_srvs::Trigger trigger_out;
        service_client_activate.call(trigger_out);

        current_state = FINISHED_STATE;
      }

      break;
    }

    break;
  }
}

//}

}  // namespace automatic_start_eagle

}  // namespace mrs_general

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_general::automatic_start_eagle::AutomaticStartEagle, nodelet::Nodelet)
