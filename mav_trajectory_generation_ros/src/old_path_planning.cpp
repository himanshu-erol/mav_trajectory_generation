
#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "visualization_msgs/Marker.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
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

#include <std_srvs/Empty.h>




namespace ob = ompl::base;
namespace og = ompl::geometric;


// Declear some global variables

trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg_prev;
float goal_x = 0.0;   //setting some default values
float goal_y = 0.0;
float goal_z = 0.0;

float cur_x = 0.0;   //setting some default values
float cur_y = 0.0;
float cur_z = 0.0;

bool flag_sub_octomap=0;
int flag_sub_goal=0;

//ROS publishers

ros::Publisher vis_pub;
ros::Publisher spline_pub;
ros::Publisher traj_pub;
ros::ServiceClient visualize_client;

std::shared_ptr<fcl::CollisionGeometry> Quadcopter(new fcl::Box(0.4, 0.4, 0.1));//0.3,0.3.0.1
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

bool isWaypointValid(const ob::ScopedState<ob::SE3StateSpace> &se3state)
{
   
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



void waypointsCallback1( trajectory_msgs::MultiDOFJointTrajectory msg)
{
    trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
    int num_waypoints = int(msg.points.size());

    std::cout << num_waypoints <<"###########################################################"<< std::endl;

    //########define vertices and that ther are 3d

    mav_trajectory_generation::Vertex::Vector vertices;
    vertices.clear();
    //$
    mav_trajectory_generation::Vertex::Vector yaw_vertices_;
    mav_trajectory_generation::Trajectory yaw_trajectory_;
    yaw_vertices_.clear();
    yaw_trajectory_.clear();
    mav_trajectory_generation::Vertex yaw(1);
    //$
    const int dimension = 3;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
    const int kDerivativeToOptimize = mav_trajectory_generation::derivative_order::ACCELERATION;
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);
    

    if (num_waypoints>1) //valid number of waypoints

        {     
        
        
        for(int index=0 ; index < num_waypoints ; index++)
        { mav_trajectory_generation::Vertex temp_point(3);
          mav_trajectory_generation::Vertex fix_point(3);
          point_msg = msg.points[index];
          //store previous waypoint
          float prev_x=0.0,prev_y=0.0,prev_z=0.0,x,y,z;
          if(index)
            {   point_msg_prev = msg.points[index-1];
                point_msg_prev.transforms.resize(1);
                prev_x = point_msg_prev.transforms[0].translation.x;
                prev_y = point_msg_prev.transforms[0].translation.y;
                prev_z = point_msg_prev.transforms[0].translation.z;
            }

          point_msg.transforms.resize(1);
          x = point_msg.transforms[0].translation.x;
          y = point_msg.transforms[0].translation.y;
          z = point_msg.transforms[0].translation.z;
     


         if(index==0)
            { temp_point.makeStartOrEnd(Eigen::Vector3d(x,y,z), derivative_to_optimize);
              vertices.push_back(temp_point);
 	      yaw.addConstraint(mav_trajectory_generation::derivative_order::ORIENTATION, 0.0); 
	      yaw_vertices_.push_back(yaw);
            }

          else if(index==num_waypoints-1)
            { 
              if(num_waypoints==2) //add a middle point
                {
                     fix_point.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d((x+prev_x)/2,(y+prev_y)/2,(z+prev_z)/2));
                     vertices.push_back(fix_point);
	  	     yaw.addConstraint(mav_trajectory_generation::derivative_order::ORIENTATION, 0.0); 
	      	     yaw_vertices_.push_back(yaw);
                }
              temp_point.makeStartOrEnd(Eigen::Vector3d(x,y,z), derivative_to_optimize);
              vertices.push_back(temp_point);
	      yaw.addConstraint(mav_trajectory_generation::derivative_order::ORIENTATION, 0.0); 
	      yaw_vertices_.push_back(yaw);

            }

          else
            { temp_point.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(x,y,z));
              vertices.push_back(temp_point);
	      yaw.addConstraint(mav_trajectory_generation::derivative_order::ORIENTATION, 0.0); 
	      yaw_vertices_.push_back(yaw);
            }

         }
            ROS_INFO("######################: %d", int(vertices.size()));

          


            //##########compute the segment times

            std::vector<double> segment_times;
            const double v_max = 0.25;
            const double a_max = 0.5;
            const double magic_fabian_constant = 6.5; // A tuning parameter.
            segment_times = estimateSegmentTimes(vertices, v_max, a_max, magic_fabian_constant);

            // #for linear optimization

            //########## creat an optimizer object and solve

            //const int N = 10;
            //mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
            //opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
            //opt.solveLinear();

            //######### obtain the polynomial segment_times

            //mav_trajectory_generation::Segment::Vector segments;
            //opt.getSegments(&segments);

            

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
            
            const int N = 20;
            mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
            opt.setupFromVertices(vertices, segment_times, kDerivativeToOptimize);
            //opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
            opt.solveLinear();
            
	    //YAW########################################

            mav_trajectory_generation::PolynomialOptimizationNonLinear<N> yaw_opt(1, parameters, false);
	    yaw_opt.setupFromVertices(yaw_vertices_, segment_times, derivative_to_optimize);
	    yaw_opt.optimize();
	    yaw_opt.getTrajectory(&yaw_trajectory_);
            //########obtain polynomial segments

            mav_trajectory_generation::Segment::Vector segments;
            opt.getSegments(&segments);


            //##########creating trajectories
            mav_trajectory_generation::Trajectory trajectory;
            opt.getTrajectory(&trajectory);
	    
        /*    //############sampling trajectories

            // Single sample:
            double sampling_time = 2.0;
            int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
            //Eigen::VectorXd sample = trajectory.evaluate(sampling_time, derivative_order);

            // Sample range:
            double t_start = 2.0;
            double t_end = 10.0;
            double dt = 0.01;
            std::vector<Eigen::VectorXd> result;
            std::vector<double> sampling_times; // Optional.
           // trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);

            mav_msgs::EigenTrajectoryPoint state;
            mav_msgs::EigenTrajectoryPoint::Vector states;

            // Single sample:
            //double sampling_time = 2.0;
            //bool success = mav_trajectory_generation::sampleTrajectoryAtTime(trajectory, sampling_time, &state);

            // Sample range:
            //double t_start = 2.0;
            double duration = 10.0;
            //double dt = 0.01;
            //success = mav_trajectory_generation::sampleTrajectoryInRange(trajectory, t_start, duration, dt, &states);

            // Whole trajectory:
            double sampling_interval = 0.01;
            //success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);
            ROS_INFO("stamp: %d", success);

   */

            //###########rviz vizualization


             int32_t shape = visualization_msgs::Marker::TEXT_VIEW_FACING;
              uint32_t action = 0;
             
             uint32_t iteration = 0;

             

             // while(1)
            {   

            visualization_msgs::MarkerArray markers;
            double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
            std::string frame_id = "world";

            // From Trajectory class:
            //mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);

            // From mav_msgs::EigenTrajectoryPoint::Vector states:
            //mav_trajectory_generation::drawMavSampledTrajectory(states, distance, frame_id, &markers);

            


	    //ALSO PUBLISH TRAJECTORY DIRECTLY TO TRAJECTORY SAMPLER
	    //planning_msgs::PolynomialTrajectory4D msg;
	    //mav_trajectory_generation::Trajectory traj_with_yaw;
	    //trajectory.getTrajectoryWithAppendedDimension(yaw_trajectory_, &traj_with_yaw);
	    //mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(traj_with_yaw, &msg);

	    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
	    vis_pub.publish(markers);
	    //path_segments_publisher_.publish(msg);

            //ros::Duration(3).sleep(); //Plan next trajectory only after three seconds
            //break;   
                //ros::spin();
                
              }

            }
}  





void plan(void)
{   
	 


	// construct the state space we are planning in
	ob::StateSpacePtr space(new ob::SE3StateSpace());

    // set the bounds for the R^3 part of SE(3)
	ob::RealVectorBounds bounds(3);
    // bounds.setLow(-1);
    // bounds.setHigh(1);
	bounds.setLow(0,-10);//-5
	bounds.setHigh(0,10);//+5
	bounds.setLow(1,-10);//-5
	bounds.setHigh(1,10);//+5
	bounds.setLow(2,-1);//-2
	bounds.setHigh(2,10);//4

	space->as<ob::SE3StateSpace>()->setBounds(bounds);

    // construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    // set state validity checking for this space
	si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));

    // create a random start state
	ob::ScopedState<ob::SE3StateSpace> start(space);
	start->setXYZ(cur_x,cur_y,cur_z);
	//start->setXYZ(0, 0, 2);
	start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	// start.random();

    // create a random goal state
	ob::ScopedState<ob::SE3StateSpace> goal(space);
	 goal->setXYZ(goal_x,goal_y,goal_z+0.2); //treshold for landing height 0.2
     //goal->setXYZ(1,-2,-1);
     //goal->setXYZ(-3.76742, 2.98793, -0.788997);
	 goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
     //goal.random();

    // create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));


std::cout << "start valid: "<<isWaypointValid(start) << std::endl;
std::cout << "goal valid: "<<isWaypointValid(goal) << std::endl;

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
	ob::PlannerStatus solved = planner->solve(4.0);


	std::cout << "Reached 2: " << std::endl;
	if (solved && isWaypointValid(start) && isWaypointValid(goal) && !pdef->hasApproximateSolution())
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
		msg.header.frame_id = "world";
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

		}

        
        


        //Path smoothing using bspline

		/*og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
		og::PathGeometric path_smooth(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
		pathBSpline->smoothBSpline(path_smooth,5);
		std::cout << "Smoothed Path" << std::endl;
		path_smooth.print(std::cout);

		
		//Publish path as markers

		visualization_msgs::Marker marker;
		marker.action = visualization_msgs::Marker::DELETEALL;
		spline_pub.publish(marker);

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
			marker.scale.x = 0.3;
			marker.scale.y = 0.3;
			marker.scale.z = 0.3;
			marker.color.a = 1.0;
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
			spline_pub.publish(marker);
			
			std::cout << "Published marker: " <<  path_smooth.getStateCount () << std::endl;  
		}*/

        //waypointsCallback1(msg);//call mav_trajectory now
        traj_pub.publish(msg);
        
        //call service to visualize trajectory
        std_srvs::Empty vis_srv;
        int s = visualize_client.call(vis_srv);
            ROS_INFO("called visualize trajectory service : %d",s);

	}
	else
		std::cout << "No solution found" << std::endl;


    
  
}

void cust_callback(const octomap_msgs::Octomap::ConstPtr &msg,const geometry_msgs::TransformStamped::ConstPtr &pose)
{
	
	ROS_INFO("IN HERE! ");

	// convert octree to collision object
	 octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
	 fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));
	fcl::CollisionObject temp((std::shared_ptr<fcl::CollisionGeometry>(tree)));
	treeObj = temp;
	std::cout<<tree_oct;
	ROS_INFO("octomap loaded!! ");
	
	double cur_x_g=pose->transform.translation.x;
	double cur_y_g=pose->transform.translation.y;
	double cur_z_g=pose->transform.translation.z;
	//converting to map frame from gazebo frame
	cur_x= cur_x_g;
	cur_y= cur_y_g;
	cur_z= cur_z_g;
      	flag_sub_goal=3;
	ROS_INFO("current pose loaded!! ");
	
    	
}




void getLandingPoint(const geometry_msgs::Pose &pose)
{ 
  goal_x = pose.position.x;
  goal_y = pose.position.y;
  goal_z = pose.position.z;

   if(flag_sub_goal==3)
    plan();

}




int main(int argc, char **argv)
{   
    //init ROS......
	ros::init(argc, argv, "octomap_planner");
    ros::NodeHandle n;

 
	
	

    //ROS Publishers.....

    vis_pub = n.advertise<visualization_msgs::MarkerArray>( "my_marker_array1" ,1,true);
    spline_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker_spline", 200 ,true);
    traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>( "planned_path" ,1,true);

    ros::Rate r(10);

    //ROS service clients

    visualize_client = n.serviceClient<std_srvs::Empty>("/raven/visualize_planned_path");

    //ROS SUbscribers..... 
	
  using namespace message_filters;
  message_filters::Subscriber<octomap_msgs::Octomap> oct_sub(n, "/octomap_binary", 1);
  message_filters::Subscriber<geometry_msgs::TransformStamped> cur_pose_sub(n, "/vicon/auk/auk", 1);
  typedef sync_policies::ApproximateTime<octomap_msgs::Octomap, geometry_msgs::TransformStamped> RetrieveSimDataPolicy;
  Synchronizer<RetrieveSimDataPolicy> sync(RetrieveSimDataPolicy(10000), oct_sub, cur_pose_sub);
  sync.registerCallback(boost::bind(&cust_callback, _1, _2));


  
    ros::Subscriber landing_pose_sub = n.subscribe("/landing_sites_np/clicked_pose", 1, getLandingPoint);

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    while(ros::ok()){


	ros::spinOnce();
	r.sleep();
	}
	return 0;
}
