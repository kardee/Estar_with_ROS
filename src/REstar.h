<<<<<<< HEAD
/*
/* ROS-ESTAR.
 *
 * Copyright (C) 2016 KARTHIKEYAN. All rights reserved.
 * License (3-Clause BSD): https://github.com/kardee/R_estar
 *
 * This code uses and is based on code from:
 *   Project: ESTAR_ROS https://https://github.com/poftwaresatent/estar2
 *   Copyright (C) 2014 Roland Philippsen. All rights reserved.
 *   License (3-Clause BSD) : https://github.com/poftwaresatent/estar2
 * ***/
=======
>>>>>>> 998b91a787fa7e928cf9ee57a5eaff78c12983cb
#include <iostream>
#include <cstdio>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <cstring>
#include <fstream>
/** including ros libraries**********************/
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
/** ********************************************/ 
#include <boost/foreach.hpp>
//#define forEach BOOST_FOREACH

/** for global path planner interface */
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

//#include <pcl_conversions/pcl_conversions.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>


#include"estar.h"
#include"cell.h"
#include"grid.h"
#include"pqueue.h"

#include <cmath>

#ifndef RESTAR_H
#define RESTAR_H

#define DIMX 50
#define DIMY 50
#define ODIST 3
#define GOALX 47
#define GOALY 46
#define STARTX 1
#define STARTY 2

using namespace std;

namespace planner
{
	class REstar : public nav_core::BaseGlobalPlanner
	{	
		public:
			ros::NodeHandle ROSNodeHandle;
			REstar(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
			REstar();
			void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
			bool makePlan(const geometry_msgs::PoseStamped& start, 
		                      const geometry_msgs::PoseStamped& goal, 
		                      std::vector<geometry_msgs::PoseStamped>& plan);			    

			~REstar();	


			estar_t estar;
			float originX;
			float originY;
			float resolution;
			costmap_2d::Costmap2DROS* costmap_ros_;
			costmap_2d::Costmap2D* costmap_;
			bool initialized_;
			int width;
			int height;
			int mapSize;	
	};

};
#endif

