/**
 * @file robot.cpp
 * @authors Manu Pillai (manump@umd.edu), Rishabh Singh (rsingh24@umd.edu)
 * @brief This file contains the library for robot class
 * @version 1.0
 * @date 2021-12-09
 * @copyright Copyright (c) 2021
 * 
 */

#include "../include/robot/robot.h"


std::array<std::array<double, 2>, 5>  fp::Robot::get_goal()
{
  ros::NodeHandle m_nh;
  std::array<XmlRpc::XmlRpcValue, 4> pos_list;
  char target_id[] = {'1', '2', '3', '4'};
  std::string aruco_lookup_locations = "/aruco_lookup_locations/target_";
  
  for (int i = 0; i <4; i++)
  {
    m_nh.getParam(aruco_lookup_locations + target_id[i], pos_list[i]);
  }

  for (int i = 0; i < 4; i++)
  {
    ROS_ASSERT(pos_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
  }
  
for (int i = 0; i < 4; i ++)
{
  for (int32_t j = 0; j < pos_list[i].size(); ++j)
  {
    ROS_ASSERT(pos_list[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    m_aruco_loc.at(i).at(j) = static_cast<double>(pos_list[i][j]);
  }
}
m_aruco_loc.at(4).at(0) = -4;   //home location for explorer 
m_aruco_loc.at(4).at(1) = 2.5;
return m_aruco_loc;

}

std::array<std::array<double, 2>, 5>  fp::Robot::get_goal(std::string m_name)
{
  
  return marker_loc;
}


void fp::Robot::fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& m_msg)
{
  ros::NodeHandle m_nh;
 
if (!m_msg->transforms.empty())
 {  ROS_INFO_STREAM("Scan succesful. Found the marker with ID "<< m_msg->transforms[0].fiducial_id);
    m_aruco_id = m_msg->transforms[0].fiducial_id;
    static tf2_ros::TransformBroadcaster brc;
    geometry_msgs::TransformStamped transformStamped;
     
    //broadcast the new frame to /tf Topic
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "marker_frame";
    //creation of marker_frame with relative to camera frame at given distance
    //which is exactly what we are seeing in the camera frame
    transformStamped.transform.translation.x = m_msg->transforms[0].transform.translation.x;
    transformStamped.transform.translation.y = m_msg->transforms[0].transform.translation.y;
    transformStamped.transform.translation.z = m_msg->transforms[0].transform.translation.z;
    transformStamped.transform.rotation.x = m_msg->transforms[0].transform.rotation.x;
    transformStamped.transform.rotation.y = m_msg->transforms[0].transform.rotation.y;
    transformStamped.transform.rotation.z = m_msg->transforms[0].transform.rotation.z;
    transformStamped.transform.rotation.w = m_msg->transforms[0].transform.rotation.w;
    

  if((transformStamped.transform.translation.x)*(transformStamped.transform.translation.x) + (transformStamped.transform.translation.y)*(transformStamped.transform.translation.y) < 4)
  {
    ROS_INFO_STREAM("Broadcasting marker "<<m_aruco_id<<" location");
    brc.sendTransform(transformStamped);
    saw_marker = true;
   
  }
  else
  {
    ROS_INFO("Ignored detected marker as it is too far.");
  }
 }
}


void fp::Robot::listen(tf2_ros::Buffer& tfBuffer)
{
  ros::NodeHandle m_nh;
  ros::Duration(1.0).sleep();
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0));
    auto trans_x = (transformStamped.transform.translation.x + m_goal.target_pose.pose.position.x)/2;
    auto trans_y = (transformStamped.transform.translation.y + m_goal.target_pose.pose.position.y)/2;
    auto trans_z = transformStamped.transform.translation.z;
    ros::Duration(4.0).sleep();
       
    
    marker_loc.at(m_aruco_id).at(0) = trans_x;
    marker_loc.at(m_aruco_id).at(1) = trans_y;

    marker_loc.at(4).at(0) = -4;
    marker_loc.at(4).at(1) =  3.5;


    ROS_INFO_STREAM("Position of marker with ID "<<m_aruco_id <<" in map frame: ["
      << trans_x << ", "
      << trans_y << ", "
      << trans_z << "]"
    );
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
}
}

void fp::Robot::move(std::array<std::array<double, 2>, 5> goal_loc)
{
 typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
 ros::NodeHandle m_nh; 

  bool goal_sent = false;
  int i = 0;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Publisher m_velocity_publisher;
  ros::Subscriber fid_reader;
  geometry_msgs::Twist msg;

  if(m_name.compare("explorer") == 0)
  {
    //buidling a velocity publisher 
    
    msg.linear.x = 0;
    msg.angular.z = 0.15;
    // move the bot with a constant angular velocity
    m_velocity_publisher = m_nh.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 10);
   
  }
  
   
   m_goal.target_pose.header.frame_id = "map";
   m_goal.target_pose.header.stamp = ros::Time::now();
   m_goal.target_pose.pose.position.x = goal_loc.at(i).at(0);
   m_goal.target_pose.pose.position.y = goal_loc.at(i).at(1);
   m_goal.target_pose.pose.orientation.w = 1.0;
   i++;  

  while (!m_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO_STREAM("Waiting for the move_base action server to come up for "<< m_name);
  }
  
  
  ros::Rate loop_rate(10);
  
  while (ros::ok()) 
  {
      if (!goal_sent) {
      ROS_INFO_STREAM("Sending goal to "<<m_name);
      m_client.sendGoal(m_goal);//this should be sent only once per goal
      m_client.waitForResult();
      goal_sent = true;
    }
    if (m_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
     
      if  (m_goal.target_pose.pose.position.x == -4)
      {
        ROS_INFO_STREAM( m_name<<" robot reached home.");
        break;
      }
  
      else
      {
       ROS_INFO_STREAM(m_name<<" robot reached goal" );
       if(m_name.compare("explorer") == 0)
      {
        
        ROS_INFO("Begin scan");
        fid_reader = m_nh.subscribe("/fiducial_transforms", 100, &fp::Robot::fiducial_callback, this);
        while (saw_marker != true)
          {
           m_velocity_publisher.publish(msg);
           ros::spinOnce();
          }
        fid_reader.shutdown();
        saw_marker = false;
        listen(tfBuffer);
      }
       
       goal_sent = false;
       
       m_goal.target_pose.header.frame_id = "map";
       m_goal.target_pose.header.stamp = ros::Time::now();
       m_goal.target_pose.pose.position.x = goal_loc.at(i).at(0);
       m_goal.target_pose.pose.position.y = goal_loc.at(i).at(1);
       m_goal.target_pose.pose.orientation.w = 1.0;
       i++;
      }  
     
    }
  
    loop_rate.sleep();
  }
 
}
