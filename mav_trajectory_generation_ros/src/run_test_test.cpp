#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>




int main(int argc, char** argv) {
  ros::init(argc, argv, "run_test");
  ros::NodeHandle nh("");
  //subscriber for waypoints
  ros::Subscriber waypoints_sub = n.subscribe("/waypoints", 1, waypointsCallback);


void waypointsCallback(trajectory_msgs::MultiDOFJointTrajectory &msg)
{

 trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
 for(loop for points)
    { if(index==0)
        { points[index].makeStartOrEnd(Eigen::Vector3d(0,0,1), derivative_to_optimize);
          vertices.push_back(points[index); 
        }

      else if(index==last)
        { points[index].makeStartOrEnd(Eigen::Vector3d(2,1,5), derivative_to_optimize);
          vertices.push_back(points[index]);

        }

      else
        { points[index].addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,2,3));
          vertices.push_back(points[index]);
        }

     }
}  

  
//########define vertices and that ther are 3d

mav_trajectory_generation::Vertex::Vector vertices;
const int dimension = 3;
const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);
mav_trajectory_generation::Vertex points(dimention)[10];

//###########create a list of vertices and add constraints

start.makeStartOrEnd(Eigen::Vector3d(0,0,1), derivative_to_optimize);
vertices.push_back(start);

middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,2,3));
vertices.push_back(middle);

end.makeStartOrEnd(Eigen::Vector3d(2,1,5), derivative_to_optimize);
vertices.push_back(end);

//##########compute the segment times

std::vector<double> segment_times;
const double v_max = 2.0;
const double a_max = 2.0;
const double magic_fabian_constant = 6.5; // A tuning parameter.
segment_times = estimateSegmentTimes(vertices, v_max, a_max, magic_fabian_constant);

/* #for linear optimization

//########## creat an optimizer object and solve

const int N = 10;
mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
opt.solveLinear();

//######### obtain the polynomial segment_times

mav_trajectory_generation::Segment::Vector segments;
opt.getSegments(&segments);

*/

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
ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "my_marker_array1" ,10);
ros::Publisher vis_pub_dummy = nh.advertise<visualization_msgs::Marker>( "my_marker_array_dummy" ,0);

 int32_t shape = visualization_msgs::Marker::TEXT_VIEW_FACING;
  uint32_t action = 0;
 
 uint32_t iteration = 0;

  while(1)
{   

visualization_msgs::Marker marker;
marker.header.frame_id = "base_link";
marker.header.stamp = ros::Time();
marker.ns = "my_namespace";
marker.id =1;
marker.type = visualization_msgs::Marker::SPHERE;
marker.action = visualization_msgs::Marker::ADD;
marker.pose.position.x = 1;
marker.pose.position.y = 1;
marker.pose.position.z = 1;
marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;
marker.scale.x = 1;
marker.scale.y = 0.1;
marker.scale.z = 0.1;
marker.color.a = 1.0; // Don't forget to set the alpha!
marker.color.r = 0.0;
marker.color.g = 1.0;
marker.color.b = 0.0;
//only if using a MESH_RESOURCE marker type:
//marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
vis_pub_dummy.publish( marker );
 
visualization_msgs::MarkerArray markers;
double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
std::string frame_id = "world";

// From Trajectory class:
mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);

// From mav_msgs::EigenTrajectoryPoint::Vector states:
mav_trajectory_generation::drawMavSampledTrajectory(states, distance, frame_id, &markers);
    
  


  

    vis_pub.publish(markers);
    
  //  ros::spin();
    
  }

  
}
