#include <roscpp.h>
#include <mbn_follower/MarkerData.h>
#include <mbn_follower/EnumerateList.h>
#include <mbn_follower/MarkerList.h>
#include <map>

class MarkerServer
{
public:
  MarkerServer()
  : _nh("~")
  {
    _list_marker_service = _nh.advertiseService("list_marker", add);
    _marker_position_service = _nh.advertiseService("marker_data", add);
    _enumerate_marker_lists = _nh.advertiseService("enumerate_list", add);
    
    std::vector<int> aux_path1;
    aux_path1.push_back(1);
    aux_path1.push_back(2);
    aux_path1.push_back(3);
    aux_path1.push_back(4);
    aux_path1.push_back(5);
    
    std::vector<int> aux_path2;
    aux_path2.push_back(5);
    aux_path2.push_back(4);
    aux_path2.push_back(3);
    aux_path2.push_back(2);
    aux_path2.push_back(1);
    _marker_map.push_back(std::make_pair("pathA1_A2",aux_path1));
    _marker_map.push_back(std::make_pair("pathA2_A1",aux_path2));
    
    mbn_follower::MarkerData stupid_data1, stupid_data2;
    stupid_data1.res.pose_ref_frame_id = "default";
    stupid_data1.res.pose.position.x = 0.0;
    stupid_data1.res.pose.position.y = 0.0;
    stupid_data1.res.pose.position.z = 0.0;
    stupid_data1.res.pose.orientation.x = 0.0;
    stupid_data1.res.pose.orientation.y = 0.0;
    stupid_data1.res.pose.orientation.z = 0.0;
    stupid_data1.res.pose.orientation.w = 1.0;
    
    stupid_data2.res.pose_ref_frame_id = "default";
    stupid_data2.res.pose.position.x = 0.0;
    stupid_data2.res.pose.position.y = 0.0;
    stupid_data2.res.pose.position.z = 0.0;
    stupid_data2.res.pose.orientation.x = 0.0;
    stupid_data2.res.pose.orientation.y = 0.0;
    stupid_data2.res.pose.orientation.z = 0.0;
    stupid_data2.res.pose.orientation.w = 1.0; 
    _marker_metadata_map.push_back(std::make_pair(1,stupid_data1));
    _marker_metadata_map.push_back(std::make_pair(5,stupid_data2));
    
  }
  
  
  
private:
  ros::NodeHandle _nh;
  ros::ServiceServer _list_marker_service;
  ros::ServiceServer _marker_position_service;
  ros::ServiceServer _enumerate_marker_lists;  
  std::map<std::string,std::vector<int> > _marker_map;
  std::map<int,mbn_follower::MarkerData> _marker_metadata_map;
  
  bool getMarkerData(mbn_follower::MarkerData::Request& req, mbn_follower::MarkerData::Response& res)
  {
    std::map<std::string,std::vector<int> >::iterator it;
    it=_marker_metadata_map.find(req.marker_id);
    if( it!=_marker_map.end() )
    {
      res.pose_ref_frame_id = it->second.pose_ref_frame_id;
      res.pose = it->pose;
      return true;
    }
    
    return false;
  }
  
  bool getEnumerateList(mbn_follower::EnumerateList::Request& req, mbn_follower::EnumerateList::Response& res)
  {
    for(std::map<std::string,std::vector<int> >::iterator it=_marker_map.begin();it!=_marker_map.end();it++)
      res.names.push_back(it->first);
      
    return true;
  }
  
  bool getMarkerList(mbn_follower::MarkerList::Request& req, mbn_follower::MarkerList::Response& res)
  {
    std::map<std::string,std::vector<int> >::iterator it;
    it=_marker_map.find(req.name);
    if( it!=_marker_map.end() )
    {
      res.list = it->second;
      return true;
    }
    
    return false;
  }
};


int main(int argc, char** argv)
{
  MarkerServer ms;
  ros::init(argc,argv,"marker_server");
  
  ros::spin();
  return 0;
}