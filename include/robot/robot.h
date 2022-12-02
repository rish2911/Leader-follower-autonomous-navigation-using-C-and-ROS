/**
 * @file robot.h
 * @authors Manu Pillai (manump@umd.edu), Rishabh Singh (rsingh24@umd.edu)
 * @brief This file contains a class to represent a robot in an environment
 * @version 1.0
 * @date 2021-12-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef ROBOT_H
#define ROBOT_H
#include <array>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <cstring>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
namespace fp {
    /**
     * @brief Class to represent a robot in an environment 
     * 
     */

    class Robot
    {
    public:
    /**
     * @brief Default Constructor for Robot object
     * 
     */
    Robot():m_client{"/explorer/move_base", true}, m_name{"explorer"}{

    }
    /**
     * @brief Construct a new Robot object
     * 
     * @param name: std::string 
     */
    Robot(std::string name):
    m_name{name},
    m_client{"/"+name+"/move_base", true}
    {

    }
    


    /**
     * @brief Listens to tf2_ros::Buffer and uses tf transform to get the coordinates of the markers in map frame.
     * Used to get goal locations for follower robot.
     * 
     * @param tfBuffer  
     */
    void listen(tf2_ros::Buffer& tfBuffer);

    /**
     * @brief Detect aruco information from the camera and publish the location and aruco ID
     * 
     * @param msg 
     */
    void fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);
    
    
      
     /**
      * @brief Get the goal array from parameter server and store it in aruco_loc
      * 
      * @return std::array<std::array<double, 2>, 5> array of goal locations including home 
      */
    std::array<std::array<double, 2>, 5> get_goal();
    /**
     * @brief Get the goal array for follower and store it in marker_loc
     * 
     * @param m_name: std::string
     * @return std::array<std::array<double, 2>, 5> 
     */
    std::array<std::array<double, 2>, 5> get_goal(std::string m_name);
    /**
     * @brief Allows the robots to move to goal location. In case of the explorer, it also turns on spot and finds the aruco marker and broadcasts its 
     * location wrt global coordinates after transformation.
     * 
     * @param goal_loc: std::array<std::array<double, 2>, 5>  
     */
    void move(std::array<std::array<double, 2>, 5> goal_loc);
   

    
    private:
    ros::NodeHandle m_nh; //ROS Node handle
    bool saw_marker{false}; // Flag for marker spotting
    int32_t m_aruco_id{}; // Index for aruco locations
    std::array<std::array<double, 2>, 5> marker_loc;  //follower goal array with the fifth element being home location
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> m_client; //move base client
    move_base_msgs::MoveBaseGoal m_goal; // move base goal
    std::string m_name; // robot name
    geometry_msgs::Twist m_msg; // used to store the detected fiducial messages
    std::array<std::array<double, 2>, 5> m_aruco_loc; //follower goal array with the fifth element being home location
    
    };
}
#endif