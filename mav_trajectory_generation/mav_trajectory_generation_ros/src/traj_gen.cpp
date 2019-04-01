#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "geometry_msgs/Pose.h"


void chatterCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  ROS_INFO("I heard");
}

int main(int argc, char **argv)

{

int i = 0, traj_size = 10;

ros::init(argc, argv, "traj_gen");

ros::NodeHandle n;

// ros::Subscriber sub = n.subscribe("/iris/ground_truth/pose", 1000, chatterCallback);

ros::Publisher traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/iris/command/trajectory", 20);

ros::Rate loop_rate(40);

while(ros::ok())
{


if(i>traj_size-1)
	i = 0;


mav_trajectory_generation::Vertex::Vector vertices;
const int dimension = 3;
const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

start.makeStartOrEnd(Eigen::Vector3d(0,0,1), derivative_to_optimize);
vertices.push_back(start);

middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0,1,1));
vertices.push_back(middle);

middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,1,1));
vertices.push_back(middle);

middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,0,1));
vertices.push_back(middle);

middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0,0,1));
vertices.push_back(middle);

end.makeStartOrEnd(Eigen::Vector3d(0,1,3), derivative_to_optimize);
vertices.push_back(end);

std::vector<double> segment_times;
const double v_max = 2.0;
const double a_max = 2.0;
segment_times = estimateSegmentTimes(vertices, v_max, a_max);

const int N = 10;
mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
opt.solveLinear();

mav_trajectory_generation::Segment::Vector segments;
opt.getSegments(&segments);


mav_trajectory_generation::Trajectory trajectory;
opt.getTrajectory(&trajectory);

// Splitting:
mav_trajectory_generation::Trajectory x_trajectory = trajectory.getTrajectoryWithSingleDimension(1);


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


static constexpr double kDefaultSamplingTime = 0.001;   //default = 0.1
geometry_msgs::Pose traj_point;
mav_msgs::EigenTrajectoryPoint::Vector flat_states;
bool suces = mav_trajectory_generation::sampleWholeTrajectory(trajectory, kDefaultSamplingTime, &flat_states);
        
traj_size = flat_states.size();        

        // conversion to pose stamped for sending to mpc controller to take control actions and sending the pose in loop
        
trajectory_msgs::MultiDOFJointTrajectory traj_msg;
mav_msgs::msgMultiDofJointTrajectoryFromEigen(flat_states[i], &traj_msg);

        // publishing trajectory to send to non linear mpc controller

traj_pub.publish(traj_msg);

visualization_msgs::MarkerArray markers;
double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
std::string frame_id = "world";

// From Trajectory class:
mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);


//vis_pub.publish(markers);

i++;

ros::spinOnce();
vertices.clear();
loop_rate.sleep();

}

return 0;

}