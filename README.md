# Bio_Facility_Automation
The current designs of bioprocess and biomanufacturing require excessive labour and manual interventions. The bio-manufacturing industry devotes 35-50% of total cost in labour, the highest among all, compared to 10-14% in the next highest industry. Besides economical reasons, there exists safety reasons to implement automation in the bio facilities especially in the wake of recent Covid-19 like situations where minimal contact is of paramount importance and the medical staff are in shortage. The proposed idea is to develop a robotic system composed of a mobile robot and manipulator arm capable of transporting material within the facility and performing the procedural tasks using the manipulator.

# Project_setup
Update system: sudo apt-get update

Install the ROS navigation stack: sudo apt-get install ros-kinetic-navigation

Create catkin workspace:

$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ~/catkin_ws
$ catkin_make
Clone the following repositories to catkin_ws/src:

$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-perception/slam_gmapping.git
$ git clone https://github.com/turtlebot/turtlebot.git
$ git clone https://github.com/turtlebot/turtlebot_interactions.git
$ git clone https://github.com/turtlebot/turtlebot_simulator.git

Install package dependencies with rosdep install [package-name]

Copy content of this repository to catkin_ws/src

Source and build the project:

$ cd ~/catkin_ws
$ source devel/setup.bash
$ catkin_make
Run ./home_service.sh in ShellScripts directory to deploy the home service robot.
