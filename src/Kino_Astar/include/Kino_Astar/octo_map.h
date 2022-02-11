
#ifndef OCTO_MAP_H
#define OCTO_MAP_H


#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <ros/ros.h>

#include <iostream>

using std::cout;
using std::endl;
namespace octo_map  {
  class OctoMap{

  private:
    ros::NodeHandle nh_;

  protected:
    ros::ServiceClient mapClient_;
		octomap::OcTree* map_;

  public:
    OctoMap(const ros::NodeHandle& nh):nh_(nh){
      this->mapClient_ = this->nh_.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
      this->updateMap();

    }

    void updateMap(){
      octomap_msgs::GetOctomap mapSrv;
      bool service_success = this->mapClient_.call(mapSrv);
      ros::Rate rate(10);
      while (not service_success and ros::ok()){
        service_success = this->mapClient_.call(mapSrv);
        ROS_INFO("[Map INFO]: Wait for Octomap Service...");
        rate.sleep();
      }
      octomap::AbstractOcTree* abtree = octomap_msgs::binaryMsgToMap(mapSrv.response.map);
      this->map_ = dynamic_cast<octomap::OcTree*>(abtree);

      // double resolution;
      // nh_.param("map/map_resolution",resolution);
      // this->map_->setResolution(resolution);
  
      cout << "[Map INFO]: Map updated!" << endl;

    }

    octomap::OcTree* getMapPtr(){
      return map_;
    }



  };
}

#endif