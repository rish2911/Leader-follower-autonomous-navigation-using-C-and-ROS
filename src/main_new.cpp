/**
 * @file main_new.cpp
 * @authors Manu Pillai (manump@umd.edu), Rishabh Singh (rsingh24@umd.edu)
 * @brief This file contains the main function, which is the executable file for this project
 * @version 1.0
 * @date 2021-12-09
 * @copyright Copyright (c) 2021
 * 
 */
#include "../include/robot/robot.h"


int main(int argc, char** argv)
{
//declaring goal arrays
std::array<std::array<double, 2>, 5> explorer_goals;
std::array<std::array<double, 2>, 5> follower_goals;   

// ros initialization
ros::init(argc, argv, "simple_navigation_goals");

//constructing explorer and follower objects
fp::Robot explorer("explorer");
fp::Robot follower("follower");

//Moving explorer and follower
explorer_goals = explorer.get_goal();
explorer.move(explorer_goals);
follower_goals = explorer.get_goal("explorer");
follower.move(follower_goals);

//shutting down 
ros::shutdown();
}
