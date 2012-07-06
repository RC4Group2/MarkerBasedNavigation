
#include <ros/ros.h>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <mbn_msgs/MarkersPoses.h>
#include <sensor_msgs/Joy.h>
#include <marker_server/MarkerList.h>
#include <marker_server/MarkerData.h>
#include <tf/transform_listener.h>

class MarkerGo {
public:
  MarkerGo(ros::NodeHandle &nh)
    : rosnode(nh),
      ac_(rosnode, "move_base")
  {
    ROS_INFO("Initializing and subscribing");
    goal_sent_ = true; // wait for joystick input
    active_path_ = 0;
    active_path_switch_time_ = ros::Time::now();

    mrkserv_client_ =
      rosnode.serviceClient<marker_server::MarkerList>("marker_server/list_marker");

    marker_server::MarkerList ml;
    ml.request.name = "pathA1_A2";
    mrkserv_client_.call(ml);
    for (size_t i = 0; i < ml.response.list.markersIDs.size(); ++i)
    {
      ROS_INFO("Got A1 -> A2 marker %zu: %i",
	       i, ml.response.list.markersIDs[i]);
      path_a1_a2_.push_back(ml.response.list.markersIDs[i]);
    }

    ml.request.name = "pathA2_A1";
    mrkserv_client_.call(ml);
    for (size_t i = 0; i < ml.response.list.markersIDs.size(); ++i)
    {
      ROS_INFO("Got A2 -> A1 marker %zu: %i",
	       i, ml.response.list.markersIDs[i]);
      path_a2_a1_.push_back(ml.response.list.markersIDs[i]);
    }

    sub_mpos_ =
      rosnode.subscribe<mbn_msgs::MarkersPoses>(
        "markers_poses_topic", 1,
        boost::bind(&MarkerGo::markers_pos_cb, this, _1));

    sub_joy_ =
      rosnode.subscribe<sensor_msgs::Joy>(
        "joy", 1,
        boost::bind(&MarkerGo::joy_cb, this, _1));


  }

  void
  move_base_done_cb(const actionlib::SimpleClientGoalState& state,
		    const move_base_msgs::MoveBaseResultConstPtr& result)
  {
    if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("*** TARGET REACHED ***");
    } else {
      ROS_ERROR("Failed to reach target: %s",
		state.text_.c_str());
    }
    //ros::shutdown();
  }

  void
  joy_cb(const sensor_msgs::JoyConstPtr &joy)
  {
    if (joy->buttons[6] > 0) {
      goal_sent_ = false;
    }
    if (joy->buttons[1] > 0) {
      if ((ros::Time::now() - active_path_switch_time_).toSec() > 2.0) {
	active_path_ = 1 - active_path_;
	ROS_INFO("Switched to path %s",
		 (active_path_ == 0) ? "A1->A2" : "A2->A1");
	active_path_switch_time_ = ros::Time::now();
      }
    }
  }

  void
  markers_pos_cb(const mbn_msgs::MarkersPosesConstPtr &markers)
  {
    if (goal_sent_)  return;

    if (markers->markersPoses.size() == 0) {
      ROS_ERROR("No marker visible");
      //ros::shutdown();
      return;
    }

    int goal_marker = -1;
    std::vector<int> &ap =
      (active_path_ == 0) ? path_a1_a2_ : path_a2_a1_;
    for (size_t i = 0; i < markers->markersPoses.size(); ++i) {
      for (unsigned int j = 0; j < ap.size(); ++j) {
	if (ap[j] == markers->markersPoses[i].marker_id) {
	  goal_marker = i;
	  break;
	}
      }
      if (goal_marker != -1)  break;
    }

    if (goal_marker == -1) {
      ROS_WARN("No suitable marker found for path");
      return;
    }

    const mbn_msgs::MarkerPose &p =
      markers->markersPoses[goal_marker];

    //tf::Pose ps;
    //tf::poseMsgToTF(p.poseWRTRobotFrame, ps);

    ROS_INFO("Sending goal (%f,%f,%f)",
	     p.poseWRTRobotFrame.position.x,
	     p.poseWRTRobotFrame.position.y,
	     p.poseWRTRobotFrame.position.z);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "/base_link";
    //tf::poseTFToMsg(next_pose, goal.target_pose.pose);
    goal.target_pose.pose = p.poseWRTRobotFrame;
    goal.target_pose.pose.orientation.x =
      goal.target_pose.pose.orientation.y =
      goal.target_pose.pose.orientation.z = 0.;
    goal.target_pose.pose.orientation.w = 1.;

    ac_.sendGoal(goal,
		 boost::bind(&MarkerGo::move_base_done_cb, this, _1, _2));
    goal_sent_ = true;
  }

 private:
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MBClient;

  ros::NodeHandle &rosnode;
  MBClient ac_;

  ros::ServiceClient mrkserv_client_;

  bool goal_sent_;

  move_base_msgs::MoveBaseActionGoal mb_goal_;

  ros::Subscriber sub_mpos_;
  ros::Subscriber sub_joy_;
  tf::TransformListener tf_;

  std::vector<int> path_a1_a2_;
  std::vector<int> path_a2_a1_;

  int active_path_;
  ros::Time active_path_switch_time_;
};

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_go");
  ros::NodeHandle rosnode;

  MarkerGo go(rosnode);

  ros::spin();

  return 0;
}

