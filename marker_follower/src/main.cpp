
#include <ros/ros.h>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <marker_follower/FollowMarkerListAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <mbn_msgs/MarkersPoses.h>
#include <mbn_common/MarkerSearcherCoordination.hpp>
#include <mbn_common/MarkerSearcherComputation.hpp>
#include <mbn_common/MarkerPathIteratorCoordination.hpp>
#include <mbn_common/MarkerPathIteratorComputation.hpp>
#include <mbn_common/MarkerPathPlannerComputation.hpp>

using namespace mbn_common;

class MarkerFollower {
public:
  MarkerFollower(ros::NodeHandle &nh)
    : rosnode(nh),
      as_(rosnode, "marker_follower",
	  boost::bind(&MarkerFollower::as_callback, this, _1),
	  false),
      ac_(rosnode, "move_base"),
      ms_coord_(&ms_comp_),
      mpi_coord_(&mpi_comp_)
  {
    //mrkserv_client_ = rosnode.serviceClient<marker_server::

    sub_mpos_ =
      rosnode.subscribe<mbn_msgs::MarkersPoses>(
        "markers_poses_topics", 1,
        boost::bind(&MarkerFollower::markers_pos_cb, this, _1));
    
    markers_a1_a2_.push_back(1);
    markers_a1_a2_.push_back(2);
    markers_a1_a2_.push_back(3);
    markers_a1_a2_.push_back(4);
    markers_a1_a2_.push_back(5);

    markers_a2_a1_.push_back(11);
    markers_a2_a1_.push_back(12);
    markers_a2_a1_.push_back(13);
    markers_a2_a1_.push_back(14);
    markers_a2_a1_.push_back(15);
  }


  void as_callback(const marker_follower::FollowMarkerListGoalConstPtr& goal)
  {
    if (goal->list_name == "A1_A2") {
      ms_comp_.setTargetMarkerIDs(markers_a1_a2_);
    } else {
      ms_comp_.setTargetMarkerIDs(markers_a2_a1_);
    }
    last_path_ = goal->list_name;
    ms_coord_.notifyTargetMarkerIDsReceived();

    goal_ = goal;
  }

  void
  markers_pos_cb(const mbn_msgs::MarkersPosesConstPtr &markers)
  {
    std::vector<int> ids;
    std::vector<tf::Pose> poses;
    for (size_t i = 0; i < markers->markersPoses.size(); ++i) {
      const mbn_msgs::MarkerPose &p = markers->markersPoses[i];
      ids.push_back(p.marker_id);

      tf::Pose ps;
      tf::poseMsgToTF(p.poseWRTRobotFrame, ps);
      poses.push_back(ps);
    }

    ms_comp_.setVisibleMarkersIDs(ids);
    ms_coord_.notifyVisibleMarkersIDsReceived();

    mpi_comp_.setDetectedMarkers(poses, ids);
    mpi_coord_.notifyDetectedMarkersReceived();
  }

  void ms_callback(const int idFound)
  {
    std::vector<int> itr_path;
    std::vector<int>::iterator spec_path_itr_begin;
    std::vector<int>::iterator spec_path_itr_end;
    
    if(last_path_ == "A1_A2")
    {
      spec_path_itr_begin =
	std::find(markers_a1_a2_.begin(), markers_a1_a2_.end(),
		  idFound);
	spec_path_itr_end = markers_a1_a2_.end();
    }
    else
      {
	spec_path_itr_begin =
	  std::find(markers_a2_a1_.begin(), markers_a2_a1_.end(),
		  idFound);
	spec_path_itr_end = markers_a2_a1_.end();
      }


    spec_path_itr_begin++;

    for(std::vector<int>::iterator itr = spec_path_itr_begin ; itr != spec_path_itr_end; ++itr)
    {
	itr_path.push_back(*itr);
    }

    mpi_comp_.setMarkerPath(itr_path);
    mpi_coord_.notifyMarkerPathReceived();
  }

 private:
  ros::NodeHandle &rosnode;
  actionlib::SimpleActionServer<marker_follower::FollowMarkerListAction> as_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
  ros::ServiceClient mrkserv_client_;
 
  MarkerSearcherCoordination     ms_coord_;
  MarkerSearcherComputation      ms_comp_;
  MarkerPathIteratorCoordination mpi_coord_;
  MarkerPathIteratorComputation  mpi_comp_;
  MarkerPathPlannerComputation   mpp_comp_;

  move_base_msgs::MoveBaseActionGoal mb_goal_;

  ros::Subscriber sub_mpos_;

  std::vector<int> markers_a1_a2_;
  std::vector<int> markers_a2_a1_;
  std::string last_path_;

  marker_follower::FollowMarkerListGoalConstPtr goal_;
};

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_follower");
  ros::NodeHandle rosnode;

  /*
  ros::Subscriber sub = rosnode.subscribe("markers_poses_topic", 1, marker_poses_cb);
  pub = rosnode.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  */

  MarkerFollower follower(rosnode);

  ros::spin();

  return 0;
}

