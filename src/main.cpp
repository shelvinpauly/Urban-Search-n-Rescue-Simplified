#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include "bot_controller.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
  bool explorer_goal_sent = false;
  bool follower_goal_sent = false;
  //declare vector for location
  std::vector<XmlRpc::XmlRpcValue> locations;//Location vector for Explorer
  
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;//Declaring node handle
  Bot_Controller controller(&nh);

  // tell the action client that we want to spin a thread by default
  MoveBaseClient explorer_client("/explorer/move_base", true);
  // tell the action client that we want to spin a thread by default
  MoveBaseClient follower_client("/follower/move_base", true);
  //Declaring the loation for explorer
  XmlRpc::XmlRpcValue location_1;
  XmlRpc::XmlRpcValue location_2;
  XmlRpc::XmlRpcValue location_3;
  XmlRpc::XmlRpcValue location_4;
  XmlRpc::XmlRpcValue location_explorer_current;
  double x = 0;
  double y=0;
  //Getting Param value
  ROS_INFO("Trying to get param value");
  nh.getParam("/aruco_lookup_locations/target_1", location_1);
  nh.getParam("/aruco_lookup_locations/target_2", location_2);
  nh.getParam("/aruco_lookup_locations/target_3", location_3);
  nh.getParam("/aruco_lookup_locations/target_4", location_4);
  nh.getParam("/aruco_lookup_locations/target_explorer", location_explorer_current);\
  ROS_INFO("Param value Recieved");

  //Checking the param values
  ROS_ASSERT(location_1.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(location_1[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
  ROS_ASSERT(location_1[1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
  ROS_ASSERT(location_2.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(location_2[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
  ROS_ASSERT(location_2[1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
  ROS_ASSERT(location_3.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(location_3[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
  ROS_ASSERT(location_3[1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
  ROS_ASSERT(location_4.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(location_4[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
  ROS_ASSERT(location_4[1].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  //Pushing to the explorer location list
  locations.push_back(location_1);
  locations.push_back(location_2);
  locations.push_back(location_3);
  locations.push_back(location_4);
  locations.push_back(location_explorer_current);
  


  // wait for the action server to come up
  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  while (!follower_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for follower");
  }
  //Declaring movebase for follower and explorer
  move_base_msgs::MoveBaseGoal explorer_goal;
  move_base_msgs::MoveBaseGoal follower_goal;



  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);

  //Declaring to check if the work is completed by the explorer
  bool is_explorer_work= false;
  bool is_follower_work= false;
  while (ros::ok()) 
  {
    if(!is_explorer_work)
    {
      for(int i=0;i<locations.size();i++)
      {

        //Setting location for move base 
        explorer_goal.target_pose.header.frame_id = "map";
        explorer_goal.target_pose.header.stamp = ros::Time::now();
        explorer_goal.target_pose.pose.position.x = static_cast<double>(locations[i][0]);//
        explorer_goal.target_pose.pose.position.y = static_cast<double>(locations[i][1]);//
        explorer_goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Sending goal for explorer");
        explorer_client.sendGoal(explorer_goal);//Sending location only once for each loop
        geometry_msgs::Twist msg;
        msg.angular.z = 0.1;//Angular twist before robot detecting the marker

        while(explorer_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) 
        {
          ROS_INFO("Moving the explorer");
          
        }
        if(explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
        {
          ROS_INFO("Explorer Reached Goal");
          
          controller.is_marker_found=false;
          
          if(i+1!=locations.size())//Checking if the robot reached inintial position
          {              
              while (!controller.is_marker_found)
              {
                
                controller.rotation_publisher.publish(msg);//Publishing the angular rotation of robot
                controller.m_initialize_subscribers();  //Subscribing fiducial marker
                ros::spinOnce();       
                
                if(controller.is_marker_found)
                {
                  msg.angular.z=0; //Angular twist after robot detected the marker
                  int pub=0;
                  while(pub<10)
                  {
                    controller.rotation_publisher.publish(msg);//Publishing the angular rotation of robot
                    pub++;
                  }
                  ROS_INFO("Marker found"); 
                  controller.m_initialize_subscribers();
                  ros::Duration(1.0).sleep();             
                  ros::spinOnce();
                  controller.listen(tfBuffer,static_cast<double>(locations[i][0]),static_cast<double>(locations[i][1])); //Listener code to get marker position with respect to global
                  controller.is_marker_found=false;
                  break;
                }
              }   
          }  
          else{
            is_explorer_work=true;  
          }          
        }
      }
    }  
    controller.sort();//Sort function according to the frame id
    if(!is_follower_work)
    {
      for(int i=0;i<controller.follower_locations.size();i++)
      {
        //Building goal for follower
        follower_goal.target_pose.header.frame_id = "map";
        follower_goal.target_pose.header.stamp = ros::Time::now();
        follower_goal.target_pose.pose.position.x = controller.follower_locations[i][1];
        follower_goal.target_pose.pose.position.y = controller.follower_locations[i][2];
        follower_goal.target_pose.pose.orientation.w =controller.follower_locations[i][3];

        ROS_INFO("Sending goal to Follower");
        follower_client.sendGoal(follower_goal);
        while(follower_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) 
        {
          ROS_INFO("Moving the Follower");
        }
        if(follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
        {
          ROS_INFO("Follower Goal Reached");
          is_follower_work=true;
          continue;
        }
      } 
    }
    loop_rate.sleep();
    ros::shutdown();
  }
}