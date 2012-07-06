
#include <ros/ros.h>
#include <mbn_msgs/MarkersPoses.h>
#include <visualization_msgs/MarkerArray.h>
#include <cstdio>
#include <cstdlib>

ros::Publisher pub;
int last_id_num = 0;


static void
marker_poses_cb(const mbn_msgs::MarkersPosesConstPtr &mm)
{
  int id_num = 0;

  visualization_msgs::MarkerArray m;

  for (unsigned int i = 0; i < mm->markersPoses.size(); ++i) {
    const mbn_msgs::MarkerPose &mp = mm->markersPoses[i];

    visualization_msgs::Marker cube;
    cube.header.frame_id = "/base_link";
    cube.header.stamp = mp.header.stamp;
    cube.ns = "markers";
    cube.id = id_num++;
    cube.type = visualization_msgs::Marker::CUBE;
    cube.action = visualization_msgs::Marker::ADD;
    cube.pose.position.x = mp.poseWRTRobotFrame.position.x;
    cube.pose.position.y = mp.poseWRTRobotFrame.position.y;
    cube.pose.position.z = mp.poseWRTRobotFrame.position.z;
    cube.pose.orientation.x = mp.poseWRTRobotFrame.orientation.x;
    cube.pose.orientation.y = mp.poseWRTRobotFrame.orientation.y;
    cube.pose.orientation.z = mp.poseWRTRobotFrame.orientation.z;
    cube.pose.orientation.w = mp.poseWRTRobotFrame.orientation.w;
    cube.scale.x = cube.scale.y = 0.04;
    cube.scale.z = 0.005;
    cube.color.r = 1.0;
    cube.color.g = 0.;
    cube.color.b = 0.;
    cube.color.a = 1.0;
    cube.lifetime = ros::Duration(1., 0);
    m.markers.push_back(cube);

    visualization_msgs::Marker text;
    text.header.frame_id = "/base_link";
    text.header.stamp = mp.header.stamp;
    text.ns = "markers";
    text.id = id_num++;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.position.x = mp.poseWRTRobotFrame.position.x;
    text.pose.position.y = mp.poseWRTRobotFrame.position.y;
    text.pose.position.z = mp.poseWRTRobotFrame.position.z + 0.1;
    text.pose.orientation.w = 1.;
    text.scale.z = 0.10; // 5cm high
    text.color.r = text.color.g = text.color.b = 1.0f;
    text.color.a = 1.0;
    text.lifetime = ros::Duration(1., 0);

    char *tmp;
    if (asprintf(&tmp, "M%i", mp.marker_id) != -1) {
      text.text = tmp;
      free(tmp);
    } else {
      text.text = "M?";
    }
    m.markers.push_back(text);
  }

  for (int i = id_num; i < last_id_num; ++i) {
    visualization_msgs::Marker delop;
    delop.header.frame_id = "/base_link";
    delop.header.stamp = ros::Time::now();
    delop.ns = "markers";
    delop.id = i;
    delop.action = visualization_msgs::Marker::DELETE;
    m.markers.push_back(delop);
  }
  last_id_num = id_num;

  pub.publish(m);
}

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "mbn_visualizer");
  ros::NodeHandle rosnode;

  ros::Subscriber sub = rosnode.subscribe("markers_poses_topic", 1, marker_poses_cb);
  pub = rosnode.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

  ros::spin();

  return 0;
}

