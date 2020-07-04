#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include <math.h>

bool picked = false;
bool dropped = false;
float threshold = 0.25;
float ends[2][3] = {{7.0, -4.5, 0.0}, {-1.5, 1.5, 0.0}};

//recieves odometry values from subscribing

void callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	//current coordinates
  float x = 0;	
  float y = 0;
  	//difference between current and the end x&y values
  float pick_x_diff = 0.0;
  float pick_y_diff = 0.0;
  float drop_x_diff = 0.0;
  float drop_y_diff = 0.0;
  	//distance between current and ends
  float pick_diff = 0.0;
  float drop_diff = 0.0;

  x = msg->pose.pose.position.x ;
  y = msg->pose.pose.position.y ;
  pick_x_diff = std::abs(x - ends[0][0]);
  pick_y_diff = std::abs(y - ends[0][1]);
  pick_diff = sqrt(pow(pick_x_diff,2) + pow(pick_y_diff,2));
  drop_x_diff = std::abs(x - ends[1][0]);
  drop_y_diff = std::abs(y - ends[1][1]);
  drop_diff = sqrt(pow(drop_x_diff,2) + pow(drop_y_diff,2));

  if (pick_x_diff < threshold && pick_y_diff < threshold)
  {
    picked = true;
    ROS_INFO("Object has been picked up!");
  }
  else if(drop_y_diff < threshold && drop_x_diff < threshold && picked)
  {
    dropped = true;
    ROS_INFO("Object is dropped");
  }
  else
  {

    ROS_INFO("curr_x: %f", x);
    ROS_INFO("curr_y: %f", y);
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Subscriber odom = n.subscribe("/odom", 1000, callback);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "add_markers";
  marker.id = 0;

  marker.type = visualization_msgs::Marker::CUBE;

  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();

  while(ros::ok())
  {

    if(!picked)
    {
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = ends[0][0];
      marker.pose.position.y = ends[0][1];
      marker.pose.position.z = 0.5;
      marker.pose.orientation = tf::createQuaternionMsgFromYaw(ends[0][2]);
      marker_pub.publish(marker);
    }
    else
    {
      ros::Duration(5.0).sleep();
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
    }
    if(dropped)
    {
      ROS_INFO("dropped");
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = ends[1][0];
      marker.pose.position.y = ends[1][1];
      marker.pose.position.z = 0.5;
      marker.pose.orientation = tf::createQuaternionMsgFromYaw(ends[1][2]);
      marker_pub.publish(marker);
      ros::Duration(5.0).sleep();
    }
    ros::spinOnce();
  }
  return 0;
}
