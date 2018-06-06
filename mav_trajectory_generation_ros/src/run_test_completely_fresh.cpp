#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>


//########define vertices and that ther are 3d

mav_trajectory_generation::Vertex::Vector vertices;
const int dimension = 3;
const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

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

NonlinearOptimizationParameters parameters;
parameters.max_iterations = 1000;
parameters.f_rel = 0.05;
parameters.x_rel = 0.1;
parameters.time_penalty = 500.0;
parameters.initial_stepsize_rel = 0.1;
parameters.inequality_constraint_tolerance = 0.1;

//#######create optimizer object and solve. (true/false) specifies if optimization run on just segment times

const int N = 10;
PolynomialOptimizationNonLinear<N> opt(dimension, parameters, false);
opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
opt.optimize();

//########obtain polynomial segments

mav_trajectory_generation::Segment::Vector segments;
opt.getPolynomialOptimizationRef().getSegments(&segments);

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
double sampling_time = 2.0;
bool success = mav_trajectory_generation::sampleTrajectoryAtTime(trajectory, sample_time, &state);

// Sample range:
double t_start = 2.0;
double duration = 10.0;
double dt = 0.01;
success = mav_trajectory_generation::sampleTrajectoryInRange(trajectory, t_start, duration, dt, &states);

// Whole trajectory:
double sampling_interval = 0.01;
success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);


//###########rviz vizualization

visualization_msgs::MarkerArray markers;
double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
std::string frame_id = "world";

// From Trajectory class:
mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);

// From mav_msgs::EigenTrajectoryPoint::Vector states:
mav_trajectory_generation::drawMavSampledTrajectory(states, distance, frame_id, &markers)
