#include <map>
#include <vector>
#include <string>

#include <ros/ros.h>

#include <marker_server/MarkerData.h>
#include <marker_server/EnumerateList.h>
#include <marker_server/MarkerList.h>
#include <yaml-cpp/yaml.h>

class MarkerServer
{
private:
  ros::NodeHandle _nh;
  ros::ServiceServer _list_marker_service;
  ros::ServiceServer _marker_position_service;
  ros::ServiceServer _enumerate_marker_lists;

  struct MarkerList {
   std::string name;
   std::vector <mbn_msgs::MarkersIDs> ids;
  };

 struct MarkerData {
   int name;
   marker_server::MarkerData::Response data;
  };

  
void operator >> (const YAML::Node& node, MarkerList& list) {
   node["name"] >> list.name;
   const YAML::Node& ids = node["ids"];
   for(unsigned i=0;i<ids.size();i++) {
      list.ids.push_back(ids[i].as<int>());
   }
}

void operator >> (const YAML::Node& node, MarkerData& mdata) {
   mdata.id = node["id"].as<int>();
   node["ref_frame"] >> mdata.pose_ref_frame_id;
   const YAML::Node& ids = node["position"];
   if(ids.size()>=3) {
      mdata.pose.position.x = ids[0].as<int>();
      mdata.pose.position.y = ids[1].as<int>();
      mdata.pose.position.z = ids[2].as<int>();
   }
   ids = node["orientation"];
   if(ids.size()>=4) {
      mdata.pose.orientation.x = ids[0].as<int>();
      mdata.pose.orientation.y = ids[0].as<int>();
      mdata.pose.orientation.z = ids[0].as<int>();
      mdata.pose.orientation.w = ids[0].as<int>();
   }
}

  typedef std::map<std::string, mbn_msgs::MarkersIDs> _marker_map_t;
  _marker_map_t _marker_map;
  
  typedef std::map<int, marker_server::MarkerData::Response> _marker_metadata_map_t;  
  _marker_metadata_map_t _marker_metadata_map;
  
  bool MarkerData(marker_server::MarkerData::Request& req, marker_server::MarkerData::Response& res)
  {
    _marker_metadata_map_t::const_iterator it;
    it=
    _marker_metadata_map.find(req.marker_id);
    if( it != _marker_metadata_map.end() )
    {
      res.pose_ref_frame_id = it->second.pose_ref_frame_id;
      res.pose = it->second.pose;
      return true;
    }
    
    return false;
  }
  
  bool EnumerateList(marker_server::EnumerateList::Request& req, marker_server::EnumerateList::Response& res)
  {
    for(_marker_map_t::iterator it = _marker_map.begin(); it != _marker_map.end(); ++it)
      res.names.push_back(it->first);
      
    return true;
  }
  
  bool MarkerList(marker_server::MarkerList::Request& req, marker_server::MarkerList::Response& res)
  {
    _marker_map_t::const_iterator it;
    it = _marker_map.find(req.name);

    if( it != _marker_map.end() )
    {
      res.list = it->second;
      return true;
    }
    
    return false;
  }

public:
  MarkerServer()
  : _nh("~")
  {
    _list_marker_service = _nh.advertiseService("list_marker", &MarkerServer::MarkerList, this);
    _marker_position_service = _nh.advertiseService("marker_data", &MarkerServer::MarkerData, this);
    _enumerate_marker_lists = _nh.advertiseService("enumerate_list", &MarkerServer::EnumerateList, this);
   
    std::string read_lists, read_data;

    if (nh.getParam("marker_lists", read_lists))
    {
	std::ifstream fin(read_lists);
	YAML::Parser parser(fin);
	YAML::Node doc;
	for(unsigned i=0;i<doc.size();i++)
	{
	      MarkerList list;
	      doc[i] >> list;
	      _marker_map[list.name].markersIDS = list.ids;
   	}
    }
    else
    {
     //TODO: remove this functionality, when the parsing is working correctly
         mbn_msgs::MarkersIDs::_markersIDs_type aux_path1;
    aux_path1.push_back(1);
    aux_path1.push_back(2);
    aux_path1.push_back(3);
    aux_path1.push_back(4);
    aux_path1.push_back(5);
    
    mbn_msgs::MarkersIDs::_markersIDs_type aux_path2;
    aux_path2.push_back(11);
    aux_path2.push_back(12);
    aux_path2.push_back(13);
    aux_path2.push_back(14);
    aux_path2.push_back(15);
     _marker_map["pathA1_A2"].markersIDs = aux_path1;
     _marker_map["pathA2_A1"].markersIDs = aux_path2;
    }
    
    if (nh.getParam("marker_data", read_data))
    {
	std::ifstream fin(read_lists);
	YAML::Parser parser(fin);
	YAML::Node doc;
	for(unsigned i=0;i<doc.size();i++)
	{
	      MarkerData mdata;
	      doc[i] >> mdata;
              _marker_metadata_map[mdata.id] = mdata.data;
   	}
    }
    else
    {
	    marker_server::MarkerData::Response stupid_data1, stupid_data2;
	    stupid_data1.pose_ref_frame_id = "default";
	    stupid_data1.pose.position.x = 0.0;
	    stupid_data1.pose.position.y = 0.0;
	    stupid_data1.pose.position.z = 0.0;
	    stupid_data1.pose.orientation.x = 0.0;
	    stupid_data1.pose.orientation.y = 0.0;
	    stupid_data1.pose.orientation.z = 0.0;
	    stupid_data1.pose.orientation.w = 1.0;
	    
	    stupid_data2.pose_ref_frame_id = "default";
	    stupid_data2.pose.position.x = 0.0;
	    stupid_data2.pose.position.y = 0.0;
	    stupid_data2.pose.position.z = 0.0;
	    stupid_data2.pose.orientation.x = 0.0;
	    stupid_data2.pose.orientation.y = 0.0;
	    stupid_data2.pose.orientation.z = 0.0;
	    stupid_data2.pose.orientation.w = 1.0; 
	    _marker_metadata_map[1] = stupid_data1;
	    _marker_metadata_map[5] = stupid_data2;
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc,argv,"marker_server");
  MarkerServer ms;
  
  ros::spin();
  return 0;
}
