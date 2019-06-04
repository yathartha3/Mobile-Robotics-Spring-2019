#include <cmath>
#include <string>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

#define D_S_GOAL (1.00) //how close to get before quadratic attraction
#define OB_INFLUENCE (1.2) //how close to get before replusion starts
#define ATTRACTION_FACTOR (-10.0*0.5)
#define REPULSION_FACTOR (0.10*0.5)
#define KP_angular (0.2)
#define KP_linear (0.5)
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const float PI = 3.1415927;
float laser_angle_min, laser_angle_increment, dist_to_goal;
bool flag = true; //First message

std::vector<float> goal_position{5.0, 5.0};
std::vector<float> robot_position{0.0, 0.0};
std::vector<float> final_force_vector{0.0, 0.0};
std::vector<float> total_repulsion_vector{0.0, 0.0};

float local_theta = 0;   // From robot to laser
float robot_theta = 0;	 // For robot's global theta

geometry_msgs::Quaternion quat_msg;
tf::Quaternion quat;

void calculate_force_vector(float &final_force_vector);
void calc_attract_forces(std::vector<float> &final);
void calc_repel_forces(std::vector<float> &final);
void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
float simple_dist(float x1, float y1, float x2, float y2);


void GroundTruthCB(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_position[0] = msg->pose.pose.position.x;
    robot_position[1] = msg->pose.pose.position.y;
    local_theta = atan2(2*((msg->pose.pose.orientation.w)*(msg->pose.pose.orientation.z)+(msg->pose.pose.orientation.x)*(msg->pose.pose.orientation.y)),1-2*((msg->pose.pose.orientation.y)*(msg->pose.pose.orientation.y)+(msg->pose.pose.orientation.z)*(msg->pose.pose.orientation.z)));


    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);
    robot_theta = yaw;// * (180.0/PI);	// In degrees
    // ROS_INFO("Theta:[%f]:", robot_theta);
    //ROS_INFO("X:[%f], Y:[%f]", robot_position[0],robot_position[1]);
}


void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{	
	int valid_obs_points = 0;
	final_force_vector[0] = 0.0;
    final_force_vector[1] = 0.0;
    total_repulsion_vector[0] = 0.0;
    total_repulsion_vector[1] = 0.0;

	dist_to_goal = simple_dist(robot_position[0], robot_position[1], goal_position[0], goal_position[1]);
    float fx = 0.0;
    float fy = 0.0;
    fx = ATTRACTION_FACTOR*(robot_position[0]-goal_position[0]);
    fy = ATTRACTION_FACTOR*(robot_position[1]-goal_position[1]);
    final_force_vector[0] = fx;
    final_force_vector[1] = fy;

	if (flag)
	{
		laser_angle_min = scan->angle_min;
		laser_angle_increment = scan->angle_increment;
		flag = false;
	}
    
    //Repulsive force
    for(int i = 0; i<scan->ranges.size(); i++)
    {	
    	valid_obs_points++;
    	if (scan->ranges[i]<OB_INFLUENCE)
    	{	
	    	float x=0; float y=0;
	    	float dist = scan->ranges[i];
	        /*
	        angle increment: 0.00158418, angle min: -0.506145, angle max: 0.506145*/
	        //std::cout<<"angle min: "<<scan->angle_min<<std::endl;
	        //std::cout<<"angle max: "<<scan->angle_max<<std::endl;
	        //std::cout<<"angle increment: "<<scan->angle_increment<<std::endl;

	        local_theta = laser_angle_min + (i * laser_angle_increment);
	        x = robot_position[0] + dist*cos(local_theta + robot_theta);  //Add the ground truth theta
	        y = robot_position[1] + dist*sin(local_theta + robot_theta);
	        // (x,y) are in global frame
	        total_repulsion_vector[0] += REPULSION_FACTOR*((1/dist)-(1/OB_INFLUENCE))*(1/(dist*dist))*((robot_position[0]-x)/dist);
	        total_repulsion_vector[1] += REPULSION_FACTOR*((1/dist)-(1/OB_INFLUENCE))*(1/(dist*dist))*((robot_position[1]-y)/dist);
	    }
    }
    final_force_vector[0]+= pow((640/valid_obs_points),1)*total_repulsion_vector[0];
    final_force_vector[1]+= pow((640/valid_obs_points),1)*total_repulsion_vector[1];
}


float simple_dist(float x1, float y1, float x2, float y2)
{
    return sqrt(((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2)));
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "avoider_node");

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);
    //ros::Subscriber odom_sub = n.subscribe("odom", 1000, OdomCallback);
    ros::Subscriber laser_sub = n.subscribe("scan", 1000, LaserCallback);
    ros::Subscriber ground_truth_sub = n.subscribe("/base_pose_ground_truth", 1000, GroundTruthCB);

    ros::Rate loop_rate(60);


    while(ros::ok())
    {
        ros::spinOnce();

        geometry_msgs::Twist vel;

        //calculate_force_vector(final_force_vector);
        float angular_vel = KP_angular*(atan2(final_force_vector[1],final_force_vector[0]) - robot_theta);
        float linear_vel = 0.1;//KP_linear*sqrt(pow(final_force_vector[0],2)+pow(final_force_vector[1],2));

        ROS_INFO("Distance_to_Goal:[%f]:", dist_to_goal);
        ROS_INFO_STREAM("Linear vel:" << linear_vel << ", Angular vel:" << angular_vel << " \n");

        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
        if (dist_to_goal>0.2) {vel.linear.x = linear_vel; vel.angular.z = angular_vel;}
       
        pub.publish(vel);

        loop_rate.sleep();
    }

    return 0;
}
