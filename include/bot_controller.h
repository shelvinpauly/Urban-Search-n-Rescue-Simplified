/**
 * @file bot_controller.h
 * @author Mothish Raj, Jai Sharma and Shelvin Pauly
 * @brief The file contains the bot_controller class
 * @version 0.1
 * @date 2021-12-13
 *
 * @copyright Copyright (c) 2021
 *
 */
/*! \mainpage Controlling the Follower and Explorer
  *
  * This project consists of the US&R operation which sends an explorer bot to an approximate location scans for a marker and sends the follower to recue the victim
  * - \subpage Broadcast "Broadcasting the frame"
  *
  * - \subpage Listen "Listening to the frame"
  *
  */
 /*! \page Broadcast
   *
   * This method creates a new  child frame to the camera frame for the ArUco marker and broadcasts to tf
   *
   */

/*! \page Listen
    *
    * This method listens to tf topic and then provides the information of the marker with respect to the map
    * 
*/
#ifndef BOT_CONTROLLER_H
#define BOT_CONTROLLER_H

#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <nav_msgs/Odometry.h>    //for nav_msgs::Odometry
#include <ros/ros.h>
#include <utility>
#include <tf/transform_datatypes.h> //to manipulate quaternions
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class Bot_Controller
{
    public:
    /**
         * @brief Construct a new bot_controller object to get all the functionalities of the class
         *
         * The robots are at explorer(-4,2.5) and follower(-4,3.5) 
         */
    // main() will need to instantiate a ROS nodehandle, then pass it to the constructor
    Bot_Controller(ros::NodeHandle* nodehandle); //nodehandle
    std::vector<std::array<double,4>> follower_locations; //Location vector for follower
    void m_initialize_subscribers();
    /**
     * @brief //Get the subscriber data from fisucial msgs
     *
     */
    void m_initialize_publishers();
    /**
     * @brief //Publishing to cmd_vel
     *
     */
    ros::Publisher rotation_publisher;//Rotation publisher for explorer
    bool is_marker_found= false;//Flag for finding marker 
    int frame_id ;//Frame Id of the aruko marker
    void broadcast(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);//Broadcasting the marker location according to the robot frame 
    void listen(tf2_ros::Buffer& tfBuffer,double robot_x,double robot_y);//Transforming marker location according to map frame
    ros::Subscriber fiducial_subscriber;
    
    void sort();
    /**
     * @brief //Sorts the locations accroding to their fiduicial ID
     *
     */
    private:
    ros::NodeHandle m_nh;  // we will need this, to pass between main() and constructor
    // these will be set up within the class constructor, hiding these ugly details

    

    
};
#endif 