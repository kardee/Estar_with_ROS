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

#include "REstar.h"
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(planner::REstar, nav_core::BaseGlobalPlanner)

int value;

namespace planner
{
	REstar::REstar()
	{
		cout << "Hello REstar planner." << endl;	
	}

	REstar::REstar(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	{
		cout << "Into REstar constructor : " << endl;
		//estar_init(&estar,dimx,dimy);
		initialize(name,costmap_ros);
	}
	
	void REstar::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	{

		if(!initialized_)
		{
			costmap_ros_ = costmap_ros;
			costmap_ = costmap_ros_->getCostmap();

			ros::NodeHandle private_nh("~/" + name);

			originX = costmap_->getOriginX();
			originY = costmap_->getOriginY();
		 
			width = costmap_->getSizeInCellsX();
			height = costmap_->getSizeInCellsY();
			resolution = costmap_->getResolution();
		
			mapSize = width * height;
		
 			
			value = 0;

			estar_init(&estar,width,height);    // calling the initialize function from estar2 library
			

			//costmap values are given here which is casted
			for(int i=0;i<width;++i)
			{	
				for(int j=0;j<height;++j)
				{
					size_t cost = static_cast<size_t>(costmap_->getCost(i,j));
					if(cost == 0.0)
					{
						estar_set_speed(&estar,i,j,1.0);
					}
					else
					{
						estar_set_speed(&estar,i,j,0.0);
					}  
				}
			}
			ROS_INFO("REstar planner width = %d and height = %d.",width,height);
			ROS_INFO("REstar planner %s is initialized successfully.",name.c_str());
			ROS_INFO("estar.pq.len = %d.",estar.pq.len);
			initialized_ = true;
		}
		else
		{	
			ROS_WARN("This planner has already been initialized..doing nothing");
		}

	}

	bool REstar::makePlan(const geometry_msgs::PoseStamped& start, 
		                      const geometry_msgs::PoseStamped& goal, 
		                      std::vector<geometry_msgs::PoseStamped>& plan)
	{

		if(!initialized_)
		{
			ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner.");
			return false;
		}
		ROS_INFO("Got a start : %.2f, %.2f and a goal : %.2f, %.2f",start.pose.position.x,
								            start.pose.position.y,
								            goal.pose.position.x,
								            goal.pose.position.y);
		plan.clear();

		if(goal.header.frame_id != costmap_ros_->getGlobalFrameID())
		{	
			ROS_ERROR("This planner as configured will only accept goals in the %s frame,but a goal was sent in the %s frame.",
			                         costmap_ros_->getGlobalFrameID().c_str(),goal.header.frame_id.c_str());
			return false;
		}

		tf::Stamped < tf::Pose > goal_tf;
		tf::Stamped < tf::Pose > start_tf;

		poseStampedMsgToTF(goal,goal_tf);
		poseStampedMsgToTF(start,start_tf);
		
		size_t startX = start.pose.position.x;
		size_t startY = start.pose.position.y;

		size_t goalX = goal.pose.position.x;
		size_t goalY = goal.pose.position.y;
		
		estar_set_goal (&estar,goalX,goalY);
		
		while(estar.pq.len != 0)
		{
			estar_propagate(&estar);
		}
		
		/*This part of the code is obtained from gestar.c for tracing back from start to goal*/
		size_t ii, jj;
		double topkey, maxknown, maxoverall;
        	estar_cell_t * cell;
		topkey = estar_pqueue_topkey (&estar.pq);
		ROS_INFO("Topkey = %.2f.",topkey);
  		maxknown = 0.0;
 		maxoverall = 0.0;
 		for (ii = 0; ii < width; ++ii)
	 	{
	  		for (jj = 0; jj < height; ++jj)
  	  		{	
   	 			cell = estar_grid_at (&estar.grid, ii, jj);
  	        		
				if (cell->rhs == cell->phi && isfinite(cell->rhs)) 
  	       			{
	           			if (0 == cell->pqi && cell->rhs <= topkey && maxknown < cell->rhs) 
		            		{
	                			maxknown = cell->rhs;
			    		}
	            			if (maxoverall < cell->rhs)
		            		{
	               				maxoverall = cell->rhs;
			    		}
		        	}	 
                	
          		}
		}
        	if (maxknown == 0.0) 
        	{    
			maxknown = 0.0001;
        	}
        	if (maxoverall == 0.0) 
        	{
			maxoverall = 0.0001;
        	}
		ROS_INFO("Maxknown = %.4f maxoverall = %.4f.",maxknown,maxoverall);
	
		cell = estar_grid_at(&estar.grid, startX,startY);
		if (0 == cell->pqi && cell->rhs <= maxknown) 
		{  
			ROS_INFO("Into tracing path.");
			double px,py,dd,dmax,ds;
			px = startX;
			py = startY;
			dmax = 1.3 * cell->rhs;
			geometry_msgs::PoseStamped next = start;
			ds = resolution/4;
			for(dd = 0.0; dd <= dmax; dd = dd + ds)
			{
                	        ROS_INFO("INSIDE FOR LOOP.");  		  
				double gx,gy,gg;
				ROS_INFO("before if statement of cell gradeint.");
				if(0 == estar_cell_calc_gradient(cell,&gx,&gy));
				{
				        ROS_INFO("Breaking becoz cell gradient is zero");
					break;
				}
				ROS_INFO("After cell gradient.");
				gg = sqrt(pow(gx,2.0) + pow(gy,2.0));
				gx = gx * ds/gg;
				gy = gy * ds/gg;
			
				px = px + gx;
				py = py + gy;
			
				next.pose.position.x = px*resolution;
				next.pose.position.y = py*resolution;
				next.pose.position.z = 0.0;
				
				next.pose.orientation.x = 0.0;
				next.pose.orientation.y = 0.0;
				next.pose.orientation.z = 0.0;
				next.pose.orientation.w = 1.0;
			
				plan.push_back(next);

				size_t ix = (size_t) rint (px);
				size_t iy = (size_t) rint (py);
	
				if(ix < 0 || ix >= width || iy < 0 || iy >= height)
				{
					ROS_INFO("Fell off the map at %2d %2d", ix,iy);
					break;
				}
				cell = estar_grid_at(&estar.grid,ix,iy);
				if(cell->flags & ESTAR_FLAG_GOAL)
				{
					plan.push_back(goal);
					ROS_INFO("Hit the goal at %2d %2d",ix,iy);
					break;
				}
			}
		
		}
		return true;
	}

	REstar::~REstar()
	{
		estar_fini(&estar); 
	}		 
};

