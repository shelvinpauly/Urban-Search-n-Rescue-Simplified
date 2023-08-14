#include "bot_controller.h"
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>

Bot_Controller::Bot_Controller(ros::NodeHandle* nodehandle) :
    m_nh{ *nodehandle }
{
    m_initialize_publishers();
    m_initialize_subscribers();
}


void Bot_Controller::m_initialize_publishers() {
    ROS_INFO("Initializing Publishers");
    rotation_publisher = m_nh.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 1000);
}

void Bot_Controller::m_initialize_subscribers() {
    ROS_INFO("Initializing Subscribers");
    fiducial_subscriber = m_nh.subscribe("/fiducial_transforms",10000,&Bot_Controller::broadcast,this); 
    
}

void Bot_Controller::broadcast(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
  //for broadcaster
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  if (!msg->transforms.empty()) 
  {
    is_marker_found=true;
    //broadcast the new frame to /tf Topic
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "my_frame";

    transformStamped.transform.translation.x = msg->transforms[0].transform.translation.x;
    transformStamped.transform.translation.y = msg->transforms[0].transform.translation.y;
    transformStamped.transform.translation.z = msg->transforms[0].transform.translation.z;
    transformStamped.transform.rotation.x = msg->transforms[0].transform.rotation.x;
    transformStamped.transform.rotation.y = msg->transforms[0].transform.rotation.y;
    transformStamped.transform.rotation.z = msg->transforms[0].transform.rotation.z;
    transformStamped.transform.rotation.w = msg->transforms[0].transform.rotation.w;
    frame_id = msg->transforms[0].fiducial_id;
    
    ROS_INFO("Broadcasting");
    br.sendTransform(transformStamped);
  }
}

void Bot_Controller:: listen(tf2_ros::Buffer& tfBuffer,double robot_x, double robot_y) {
  //for listener
  geometry_msgs::TransformStamped transformStamped;
  try {
    ROS_INFO("Listening");
    transformStamped = tfBuffer.lookupTransform("map", "my_frame", ros::Time(0));
    auto trans_x = transformStamped.transform.translation.x;
    auto trans_y = transformStamped.transform.translation.y;
    auto trans_z = transformStamped.transform.translation.z;
    int c = frame_id;
    ROS_INFO("Transformed");
    trans_x= (trans_x+robot_x)/2;//Getting mid point between robot and maker
    trans_y=(trans_y+robot_y)/2;//Getting mid point between robot and maker
    ROS_INFO_STREAM("Position of IDD in map frame: ["
      << trans_x << ","
      << trans_y << ","
      << trans_z << "]"
    );
    if(is_marker_found)
    {
      ROS_INFO("Added to the follower");
      std::array<double,4> fiducial_location {c,trans_x,trans_y,trans_z};  
      follower_locations.push_back(fiducial_location);
    }

  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

void Bot_Controller:: sort()
{
  for(int i=0;i<follower_locations.size()-1;i++){
    for(int j=0;j< follower_locations.size()-i-1;j++){
      if(follower_locations[j][0]>follower_locations[j+1][0])
      {
        std::array<double,4> temp= follower_locations[j];
        follower_locations[j] = follower_locations[j+1];
        follower_locations[j+1] = temp;
      }
    }
  }

  //Adding initial position of follower 
  std::array<double,4> follower_initial_position {-100.0,-4,3.5,1}; 
  follower_locations.push_back(follower_initial_position);
}






