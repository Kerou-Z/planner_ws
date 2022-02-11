#include <ros/ros.h>
#include <Kino_Astar/kino_astar.h>
#include <Kino_Astar/nonuniform_bspline.h>
#include <Kino_Astar/octo_map.h>
#include <Kino_Astar/visualization.h>
#include <geometry_msgs/PoseStamped.h>

using std::cout;
using std::endl;

bool newMsg = false;
Eigen::Vector3d newPoint {0, 0, 1.0};

void clickedPointCB(const geometry_msgs::PoseStamped::ConstPtr& cp){
	newPoint[0] = cp->pose.position.x;
	newPoint[1] = cp->pose.position.y;
	newPoint[2] = 1.0; // set height to be 1.0 m
	newMsg = true;
}



int main(int argc, char** argv){
	ros::init(argc, argv, "plan_manage_node");
	ros::NodeHandle nh;

  //octomap
  octo_map::OctoMap map(nh);

  //visualization
  visualization::Vis vis(nh);
  vis.startVisModule();

	


	// subscriber for clicked start and goal:
	ros::Subscriber clickedPointSub = nh.subscribe("/move_base_simple/goal", 1000, clickedPointCB);

	// Parameters for planner:
  kino_planner::KinodynamicAstar plan;
  plan.setParam(nh);
  plan.init();

	int countLoop = 0;
	ros::Rate r(10);

	Eigen::Vector3d start_pt;
	Eigen::Vector3d end_pt;
	while (ros::ok()){
		cout << "----------------------------------------------------" << endl;
		cout << "[Manager Node]: Request No. " << countLoop+1 << endl;
		cout << "[Manager Node]: Wait for start point..." << endl;
		while (ros::ok()){
			if (newMsg){
				start_pt = newPoint;
				newMsg = false;
				cout << "[Manager Node]: start point OK. (" << start_pt[0] << " " << start_pt[1] << " " << start_pt[2] << ")" << endl;
				break;
			}
			ros::spinOnce();
			r.sleep();
		}

		cout << "[Manager Node]: Wait for goal point..." << endl;
		while (ros::ok()){
			if (newMsg){
				end_pt = newPoint;
				newMsg = false;
				cout << "[Manager Node]: goal point OK. (" << end_pt[0] << " " << end_pt[1] << " " << end_pt[2] << ")" << endl;
				break;
			}
			ros::spinOnce();
			r.sleep();
		}

    map.updateMap();
    octomap::OcTree* map_ptr = map.getMapPtr();
    plan.setEnvironment(map_ptr);

		Eigen::Vector3d start_v(0,0,0);
    Eigen::Vector3d start_a(0,0,0);
    Eigen::Vector3d end_v(0,0,0);
    bool plan_init = true;
    bool dynamic = true;
    double time_start = 0.0;
    double ctrl_pt_dist = 0.5;
    double max_vel;
		nh.param("search/max_vel", max_vel, 1.0);
    double max_acc = 2.0;
    double max_jerk = 4;

		char result = plan.search(start_pt, start_v, start_a, end_pt, end_v, plan_init, dynamic, time_start);
    
    vector<Eigen::Vector3d> traj = plan.getKinoTraj(0.5);


		// for (auto t:traj) {
		// 	octomap::OcTreeNode* nptr = map_ptr->search(octomap::point3d(t(0), t(1),t(2)));
		// 	if (nptr){
		// 		cout << map_ptr->isNodeOccupied(nptr) <<endl;
		// 	} else {
		// 		cout << "unknown"<<endl;
		// 	}

		// }

		double ts = ctrl_pt_dist / max_vel;
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    plan.getSamples(ts, point_set, start_end_derivatives);

    Eigen::MatrixXd ctrl_pts;
    kino_planner::NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
    kino_planner::NonUniformBspline init(ctrl_pts, 3, ts);



		cout << "[Manager Node]: Start visualization" << endl;
    
		vis.updatePathVisVec(traj);

		vis.updateSplineVisVec(point_set);
		vis.publishVisMsg();

		cout << "[Manager Node]: End visualization" << endl;

		++countLoop;
		cout << "----------------------------------------------------" << endl;
	}
	return 0;
}