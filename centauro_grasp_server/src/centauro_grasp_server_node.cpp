

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include "moveit_msgs/Grasp.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>

#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "centauro_grasp_server/GraspServerGrasp.h"
#include "centauro_ur5_moveit_client/MoveStamped.h"

ros::ServiceClient move_stamped_client;
bool start_grasp(centauro_grasp_server::GraspServerGrasp::Request  &req, centauro_grasp_server::GraspServerGrasp::Response &res){
	centauro_ur5_moveit_client::MoveStamped srv;
	geometry_msgs::PoseStamped prova;
	ros::Time now=ros::Time::now();
	prova.header.stamp= now;
	prova.header.frame_id="world";
	prova.pose.position.x=0.20;
	prova.pose.position.y=0.20;
	prova.pose.position.z=-0.62;
	prova.pose.orientation.x=0.00274242;
	prova.pose.orientation.y=0.709751;
	prova.pose.orientation.z=0.00274242;
	prova.pose.orientation.w=0.704441;

	srv.request.pose=prova;
	srv.request.eelink="suction";

	ROS_INFO("Waiting for server response");

	if(move_stamped_client.call(srv)){
		
    		if(srv.response.success){
			ROS_INFO("Action successfull");
			res.success=true;
			res.message="ok";
			return true;
			
		}
		else {
			ROS_ERROR("Action Failed");
			res.success=false;
			res.message="no";
			return false;
		}
  	}
  	else{
    		ROS_ERROR("Failed to call service start_grasp_server");
    		return false;
  	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "centauro_grasp_server");
	ros::NodeHandle n;
	ros::NodeHandle n1;
	ros::ServiceServer srv1 = n.advertiseService("start_grasp_server", start_grasp);
	move_stamped_client = n1.serviceClient<centauro_ur5_moveit_client::MoveStamped>("move_stamped");
	
	ros::Rate loop_rate(10);


	while(ros::ok()){
		

	}


}
