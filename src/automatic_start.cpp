// some ros includes
#include <ros/ros.h>
#include <ros/package.h>
#include <mutex>
#include <mavros_msgs/State.h>
#include <mrs_msgs/ChangeState.h>
// subscribers and publishers
ros::Subscriber global_odom_subscriber;
ros::Publisher rtk_publisher;

// publisher rate
double safety_timeout_ = 5.0;

ros::Time armed_time;
bool armed = false;
std::mutex mutex_armed;

ros::Time offboard_time;
bool offboard = false;
std::mutex mutex_offboard;

void stateCallback(const mavros_msgs::StateConstPtr &msg) {

  mutex_armed.lock();
  {
    // check armed state
    if (armed == false) {

      // if armed state changed to true, please "start the clock"
      if (msg->armed > 0) {

        armed = true;
        armed_time = ros::Time::now();
      }

      // if we were armed previously
    } else if (armed == true) {

      // and we are not really now
      if (msg->armed == 0) {

        armed = false;
      }
    }
  }
  mutex_armed.unlock();

  mutex_offboard.lock();
  {
    // check offboard state
    if (offboard == false) {

      // if offboard state changed to true, please "start the clock"
      if (msg->mode.compare(std::string("OFFBOARD")) == 0) {

        offboard = true;
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
  mutex_offboard.unlock();
}

int main(int argc, char** argv) {

  // initialize node and create no handle
  ros::init(argc, argv, "automatic_start");
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  mutex_armed.lock();
  {
    armed = false;
    armed_time = ros::Time::now();
  }
  mutex_armed.unlock();

  mutex_offboard.lock();
  {
    offboard = false;
    offboard_time = ros::Time::now();
  }
  mutex_offboard.unlock();

  nh_.param("safety_timeout", safety_timeout_, 5.0);

  // subscriber for rc transmitter
  ros::Subscriber sub_state = nh_.subscribe("mavros_state", 1, stateCallback, ros::TransportHints().tcpNoDelay());

  // service client for starting the state machine
  ros::ServiceClient start_service_client = nh_.serviceClient<mrs_msgs::ChangeState>("change_state");

  ros::Rate r(30);

  while (ros::ok()) {

    mutex_armed.lock();
    mutex_offboard.lock();
    {
      if (armed && offboard) {

        double armed_time_diff = (ros::Time::now() - armed_time).toSec();
        double offboard_time_diff = (ros::Time::now() - offboard_time).toSec();

        if ((armed_time_diff > safety_timeout_) && (offboard_time_diff > safety_timeout_)) {

          ROS_ERROR("STARTING THE STATE MACHINE!!!!");

          mrs_msgs::ChangeState msg;

          msg.request.state_id = 1;

          start_service_client.call(msg);

          ros::shutdown();
        } else {

          double min = (armed_time_diff < offboard_time_diff) ? armed_time_diff : offboard_time_diff;

          ROS_WARN_THROTTLE(1, "Starting in %1.0f", (safety_timeout_ - min));
        }
      }
    }
    mutex_offboard.unlock();
    mutex_armed.unlock();

    ros::spinOnce();

    r.sleep();
  }

  return 0;
}
