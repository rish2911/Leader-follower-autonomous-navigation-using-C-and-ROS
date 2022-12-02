# Prerequisites
- Execute [install.bash](../script/install.bash) to install all necessary packages.
# Building Package
- Unzip the package [final project](../../final_project) to desired workspace (below example is for catkin_ws). Run catkin clean and catkin build as shown below.

```bash
cp -r final_project ~/catkin_ws/src #copy packages into workspace
cd ~/catkin_ws/ #navigate into the src folder of the catkin_ws folder
catkin clean
catkin build final_project
```

# Start application
- To run the ﬁnal project, run these two commands:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch final_project multiple_robots.launch
```

- This command will start Gazebo and RViz, will set parameters on the Parameter Server, spawn robots, and do topic remappings. In a new terminal, run:

```bash
source ~/catkin_ws/devel/setup.bash
rosrun final_project final_project_node 
```

-This command will run our node.
