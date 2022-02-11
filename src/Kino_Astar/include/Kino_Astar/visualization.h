#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <limits>
#include <visualization_msgs/MarkerArray.h>

namespace visualization
{
  class Vis
  {
  private:
    ros::NodeHandle nh_;
    visualization_msgs::MarkerArray pathVisMsg_;
    std::vector<visualization_msgs::Marker> pathVisVec_; // update this
    visualization_msgs::MarkerArray splineVisMsg_;
    std::vector<visualization_msgs::Marker> splineVisVec_; // update this
    bool visPath_;
    bool visSpline_;

    ros::Publisher pathVisPub_;
    ros::Publisher splineVisPub_;

  public:
    // std::thread pathVisWorker_;
    Vis(ros::NodeHandle nh) : nh_(nh)
    {
      visPath_ = true;
      visSpline_ = true;
    }

    void startVisModule()
    {

      if (this->visPath_)
      {
        this->pathVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/kino_planned_path", 10);
        // this->pathVisWorker_ = std::thread(&publishPathVisMsg, this);
      }
      if (this->visSpline_)
      {
        this->splineVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/spline_planned_path", 10);
        // this->pathVisWorker_ = std::thread(&publishPathVisMsg, this);
      }
    }

    void updatePathVisVec(const std::vector<Eigen::Vector3d> &plan)
    {
      pathVisVec_.clear();
      visualization_msgs::Marker waypoint;
      visualization_msgs::Marker line;
      geometry_msgs::Point p1, p2;
      std::vector<geometry_msgs::Point> lineVec;
      for (int i = 0; i < plan.size(); ++i)
      {
        Eigen::Vector3d currentPoint = plan[i];
        if (i != plan.size() - 1)
        {
          Eigen::Vector3d nextPoint = plan[i + 1];
          p1.x = currentPoint[0];
          p1.y = currentPoint[1];
          p1.z = currentPoint[2];
          p2.x = nextPoint[0];
          p2.y = nextPoint[1];
          p2.z = nextPoint[2];
          lineVec.push_back(p1);
          lineVec.push_back(p2);
        }
        // waypoint
        waypoint.header.frame_id = "map";
        waypoint.id = 1 + i;
        waypoint.ns = "kino_path";
        waypoint.type = visualization_msgs::Marker::SPHERE;
        waypoint.pose.position.x = currentPoint[0];
        waypoint.pose.position.y = currentPoint[1];
        waypoint.pose.position.z = currentPoint[2];
        waypoint.lifetime = ros::Duration(0.5);
        waypoint.scale.x = 0.08;
        waypoint.scale.y = 0.08;
        waypoint.scale.z = 0.08;
        waypoint.color.a = 0.8;
        waypoint.color.r = 0.4;
        waypoint.color.g = 0.1;
        waypoint.color.b = 0.9;
        this->pathVisVec_.push_back(waypoint);
      }
      line.header.frame_id = "map";
      line.points = lineVec;
      line.ns = "kino_path";
      line.id = 0;
      line.type = visualization_msgs::Marker::LINE_LIST;
      line.lifetime = ros::Duration(0.5);
      line.scale.x = 0.04;
      line.scale.y = 0.04;
      line.scale.z = 0.04;
      line.color.a = 1.0;
      line.color.r = 0.5;
      line.color.g = 0.1;
      line.color.b = 1;
      this->pathVisVec_.push_back(line);
      this->pathVisMsg_.markers = this->pathVisVec_;
    }

    void publishVisMsg()
    {

      ros::Rate rate(20);
      while (ros::ok())
      {
        if (this->visPath_)
        {
          this->pathVisPub_.publish(this->pathVisMsg_);
        }
        if (this->visSpline_)
        {
          this->splineVisPub_.publish(this->splineVisMsg_);
        }
      }
    }

    void updateSplineVisVec(const std::vector<Eigen::Vector3d> &plan)
    {
      splineVisVec_.clear();
      visualization_msgs::Marker waypoint;
      visualization_msgs::Marker line;
      geometry_msgs::Point p1, p2;
      std::vector<geometry_msgs::Point> lineVec;
      for (int i = 0; i < plan.size(); ++i)
      {
        Eigen::Vector3d currentPoint = plan[i];
        if (i != plan.size() - 1)
        {
          Eigen::Vector3d nextPoint = plan[i + 1];
          p1.x = currentPoint[0];
          p1.y = currentPoint[1];
          p1.z = currentPoint[2];
          p2.x = nextPoint[0];
          p2.y = nextPoint[1];
          p2.z = nextPoint[2];
          lineVec.push_back(p1);
          lineVec.push_back(p2);
        }
        // waypoint
        waypoint.header.frame_id = "map";
        waypoint.id = 1 + i;
        waypoint.ns = "Bspline_path";
        waypoint.type = visualization_msgs::Marker::SPHERE;
        waypoint.pose.position.x = currentPoint[0];
        waypoint.pose.position.y = currentPoint[1];
        waypoint.pose.position.z = currentPoint[2];
        waypoint.lifetime = ros::Duration(0.5);
        waypoint.scale.x = 0.08;
        waypoint.scale.y = 0.08;
        waypoint.scale.z = 0.08;
        waypoint.color.a = 0.7;
        waypoint.color.r = 1;
        waypoint.color.g = 0.7;
        waypoint.color.b = 0.3;
        this->splineVisVec_.push_back(waypoint);
      }
      line.header.frame_id = "map";
      line.points = lineVec;
      line.ns = "Bspline_path";
      line.id = 0;
      line.type = visualization_msgs::Marker::LINE_LIST;
      line.lifetime = ros::Duration(0.5);
      line.scale.x = 0.04;
      line.scale.y = 0.04;
      line.scale.z = 0.04;
      line.color.a = 0.5;
      line.color.r = 1;
      line.color.g = 0.5;
      line.color.b = 0.3;
      this->splineVisVec_.push_back(line);
      this->splineVisMsg_.markers = this->splineVisVec_;
    }
  };
}

#endif