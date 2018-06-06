 /*
 * Copyright 2017 Ayush Gaud 
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include "visualization_msgs/Marker.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;


// Declear some global variables

//ROS publishers
ros::Publisher vis_pub;
ros::Publisher vis_pub_final;
ros::Publisher traj_pub;

std::shared_ptr<fcl::CollisionGeometry> Quadcopter(new fcl::Box(0.3, 0.3, 0.1));
fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.1)));
fcl::CollisionObject treeObj((std::shared_ptr<fcl::CollisionGeometry>(tree)));
fcl::CollisionObject aircraftObject(Quadcopter);

bool isStateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect
	const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
	const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
	const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    // check validity of state Fdefined by pos & rot
	fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
	fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
	aircraftObject.setTransform(rotation, translation);
	fcl::CollisionRequest requestType(1,false,1,false);
	fcl::CollisionResult collisionResult;
	fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

	return(!collisionResult.isCollision());
}

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
	// obj->setCostThreshold(ob::Cost(1.51));
	return obj;
}

ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
{
	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
	obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
	return obj;
}
void plan(void)
{   //########define vertices and that ther are 3d

    mav_trajectory_generation::Vertex::Vector vertices;
    const int dimension = 3;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);
    mav_trajectory_generation::Vertex points(dimention)[10];







	// construct the state space we are planning in
	ob::StateSpacePtr space(new ob::SE3StateSpace());

    // set the bounds for the R^3 part of SE(3)
	ob::RealVectorBounds bounds(3);
    // bounds.setLow(-1);
    // bounds.setHigh(1);
	bounds.setLow(0,-10);
	bounds.setHigh(0,10);
	bounds.setLow(1,-10);
	bounds.setHigh(1,10);
	bounds.setLow(2,-10);
	bounds.setHigh(2,10);

	space->as<ob::SE3StateSpace>()->setBounds(bounds);

    // construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    // set state validity checking for this space
	si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));

    // create a random start state
	ob::ScopedState<ob::SE3StateSpace> start(space);
	start->setXYZ(1,1,1);
	start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	// start.random();

    // create a random goal state
	ob::ScopedState<ob::SE3StateSpace> goal(space);
	 goal->setXYZ(1,1,-3);
	 goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	//goal.random();

    // create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    // set the start and goal states
	pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
	ob::PlannerPtr planner(new og::RRTstar(si));

    // set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
	planner->setup();


    // print the settings for this space
	si->printSettings(std::cout);

    // print the problem settings
	pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
	ob::PlannerStatus solved = planner->solve(1.0);


	std::cout << "Reached 2: " << std::endl;
	if (solved)
	{
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
		std::cout << "Found solution:" << std::endl;
		ob::PathPtr path = pdef->getSolutionPath();
		og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
		pth->printAsMatrix(std::cout);
        // print the path to screen
        // path->print(std::cout);
		trajectory_msgs::MultiDOFJointTrajectory msg;
		trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;

		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "map";
		msg.joint_names.clear();
		msg.points.clear();
		msg.joint_names.push_back("Quadcopter");
		
		for (std::size_t path_idx = 0; path_idx < pth->getStateCount (); path_idx++)
		{
			const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

            // extract the first component of the state and cast it to what we expect
			const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // extract the second component of the state and cast it to what we expect
			const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

			point_msg.time_from_start.fromSec(ros::Time::now().toSec());
			point_msg.transforms.resize(1);

			point_msg.transforms[0].translation.x= pos->values[0];
			point_msg.transforms[0].translation.y = pos->values[1];
			point_msg.transforms[0].translation.z = pos->values[2];

			point_msg.transforms[0].rotation.x = rot->x;
			point_msg.transforms[0].rotation.y = rot->y;
			point_msg.transforms[0].rotation.z = rot->z;
			point_msg.transforms[0].rotation.w = rot->w;

			msg.points.push_back(point_msg);

            //###########create a list of vertices and add constraints

            if(path_idx==0)
              { points[path_idx].makeStartOrEnd(Eigen::Vector3d(pos->values[0],pos->values[1],pos->values[2]), derivative_to_optimize);
                vertices.push_back(points[path_idx]); 
              }

            else if(path_idx== (pth->getStateCount ()-1))
              { points[path_idx].makeStartOrEnd(Eigen::Vector3d(pos->values[0],pos->values[1],pos->values[2]), derivative_to_optimize);
                vertices.push_back(points[path_idx]);

              }

            else
              { points[path_idx].addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(pos->values[0],pos->values[1],pos->values[2]));
                vertices.push_back(points[path_idx]);
              }

		}

           
		traj_pub.publish(msg);


        //##########compute the segment times

        std::vector<double> segment_times;
        const double v_max = 2.0;
        const double a_max = 2.0;
        const double magic_fabian_constant = 6.5; // A tuning parameter.
        segment_times = estimateSegmentTimes(vertices, v_max, a_max, magic_fabian_constant);


        //#for non linear Polynomial Optimization

        //#######set the parameters for nonlinear Optimization

        mav_trajectory_generation::NonlinearOptimizationParameters parameters;
        parameters.max_iterations = 1000;
        parameters.f_rel = 0.05;
        parameters.x_rel = 0.1;
        parameters.time_penalty = 500.0;
        parameters.initial_stepsize_rel = 0.1;
        parameters.inequality_constraint_tolerance = 0.1;

        //#######create optimizer object and solve. (true/false) specifies if optimization run on just segment times

        const int N = 10;
        mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters, false);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
        opt.optimize();



        //########obtain polynomial segments

        mav_trajectory_generation::Segment::Vector segments;
        opt.getPolynomialOptimizationRef().getSegments(&segments);



        //##########creating trajectories
        mav_trajectory_generation::Trajectory trajectory;
        opt.getTrajectory(&trajectory);

        //############sampling trajectories

        // Single sample:
        double sampling_time = 2.0;
        int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
        Eigen::VectorXd sample = trajectory.evaluate(sampling_time, derivative_order);

        // Sample range:
        double t_start = 2.0;
        double t_end = 10.0;
        double dt = 0.01;
        std::vector<Eigen::VectorXd> result;
        std::vector<double> sampling_times; // Optional.
        trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);

        mav_msgs::EigenTrajectoryPoint state;
        mav_msgs::EigenTrajectoryPoint::Vector states;

        // Single sample:
        //double sampling_time = 2.0;
        bool success = mav_trajectory_generation::sampleTrajectoryAtTime(trajectory, sampling_time, &state);

        // Sample range:
        //double t_start = 2.0;
        double duration = 10.0;
        //double dt = 0.01;
        success = mav_trajectory_generation::sampleTrajectoryInRange(trajectory, t_start, duration, dt, &states);

        // Whole trajectory:
        double sampling_interval = 0.01;
        success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);
        ROS_INFO("stamp: %d", success);



        //###########rviz vizualization
        
         int32_t shape = visualization_msgs::Marker::TEXT_VIEW_FACING;
          uint32_t action = 0;
         
         uint32_t iteration = 0;

        // while(1)
        //{   

        
        visualization_msgs::MarkerArray markers;
        double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
        std::string frame_id = "world";

        // From Trajectory class:
        mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);

        // From mav_msgs::EigenTrajectoryPoint::Vector states:
        mav_trajectory_generation::drawMavSampledTrajectory(states, distance, frame_id, &markers);
            
          


          

          vis_pub_final.publish(markers);
         //}


		// Clear memory
		// pdef->clearSolutionPaths();

		///*
        //Path smoothing using bspline

		og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
		og::PathGeometric path_smooth(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
		pathBSpline->smoothBSpline(path_smooth,3);
		std::cout << "Smoothed Path" << std::endl;
		path_smooth.print(std::cout);

		
		//Publish path as markers

		visualization_msgs::Marker marker;
		marker.action = visualization_msgs::Marker::DELETEALL;
		//vis_pub.publish(marker);

		for (std::size_t idx = 0; idx < path_smooth.getStateCount (); idx++)
		{
                // cast the abstract state type to the type we expect
			const ob::SE3StateSpace::StateType *se3state = path_smooth.getState(idx)->as<ob::SE3StateSpace::StateType>();

            // extract the first component of the state and cast it to what we expect
			const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // extract the second component of the state and cast it to what we expect
			const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
			
			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time();
			marker.ns = "path";
			marker.id = idx;
			marker.type = visualization_msgs::Marker::CUBE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = pos->values[0];
			marker.pose.position.y = pos->values[1];
			marker.pose.position.z = pos->values[2];
			marker.pose.orientation.x = rot->x;
			marker.pose.orientation.y = rot->y;
			marker.pose.orientation.z = rot->z;
			marker.pose.orientation.w = rot->w;
			marker.scale.x = 0.5;
			marker.scale.y = 0.5;
			marker.scale.z = 0.5;
			marker.color.a = 1.0;
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
			vis_pub.publish(marker);
			// ros::Duration(0.1).sleep();
			std::cout << "Published marker: " << idx << std::endl;  
		}
		//*/

	}
	else
		std::cout << "No solution found" << std::endl;
}

void octomapCallback(const octomap_msgs::Octomap &msg)
{


    //loading octree from binary
	const std::string filename = "/home/valada/mapfile.bt";
	octomap::OcTree temp_tree(0.1);
	temp_tree.readBinary(filename);
	fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(&temp_tree));
	

	// convert octree to collision object
	// octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(msg));
	// fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));
	fcl::CollisionObject temp((std::shared_ptr<fcl::CollisionGeometry>(tree)));
	treeObj = temp;
	plan();

	// ros::Duration(10).sleep(); //Plan once every ten seconds
}

void octomap_load_and_plan()
{


    //loading octree from binary
	const std::string filename = "/home/valada/mapfile.bt";
	octomap::OcTree temp_tree(0.1);
	temp_tree.readBinary(filename);
	fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(&temp_tree));
	

	// convert octree to collision object
	// octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(msg));
	// fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));
	fcl::CollisionObject temp((std::shared_ptr<fcl::CollisionGeometry>(tree)));
	treeObj = temp;
	plan();

	// ros::Duration(10).sleep(); //Plan once every ten seconds
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "run_test");
	ros::NodeHandle nh("");
    vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints",10);
    vis_pub_final = nh.advertise<visualization_msgs::MarkerArray>( "my_marker_array1" ,10);


	//ros::Subscriber octree_sub = n.subscribe("/octomap_binary", 1, octomapCallback);
    octomap_load_and_plan();
	
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

	ros::spin();

	return 0;
}
