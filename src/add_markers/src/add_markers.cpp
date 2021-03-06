#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
enum State { 
  MOVING_TO_PICKUP_ZONE,
  PICKIN_UP,
  MOVING_TO_DROP_ZONE,
  WAIT
};
State robotState = MOVING_TO_PICKUP_ZONE;
bool isRobotWait = true;

visualization_msgs::Marker marker;
ros::Publisher marker_pub; 

double pickup_x = 1.97;
double pickup_y = -2.02;

double drop_x = -0.32;
double drop_y = -4.07;

double last_x = 0;
double last_y = 0;
double last_w = 0;
bool checkRobotPos = false;

double aml_x = 0; 
double aml_y = 0; 


double get_distance(double x1, double y1, double x2, double y2) {
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

bool isReachedGoal(double goal_x, double goal_y, double current_x, double current_y) {
  double distance = get_distance(goal_x, goal_y, current_x, current_y);
  //ROS_INFO("distance %f ", distance);
  return distance <= 0.5; 
}


void process_robot_position(double current_x, double current_y) 
{
 switch(robotState) {
   case MOVING_TO_PICKUP_ZONE:
        //ROS_INFO("Movin to pickup (%f;%f)  (%f;%f) %s", current_x, current_y,pickup_x, pickup_y, isRobotWait?"true":"false" );
        
        if (isReachedGoal(pickup_x, pickup_y, current_x, current_y) && isRobotWait) {
	  ROS_INFO("Pickup zone reached....");
          robotState = PICKIN_UP;
          
          marker.action = visualization_msgs::Marker::DELETE;
          marker_pub.publish(marker);
        }

	break;
   case PICKIN_UP:
        //ROS_INFO("Pickin up (%f;%f)  (%f;%f) %s", current_x, current_y,pickup_x, pickup_y, isRobotWait?"true":"false");
     
        if (isRobotWait) {
	  ROS_INFO("Pickin up....");
        } else {
	  ROS_INFO("Moving to drop zone....");
          robotState = MOVING_TO_DROP_ZONE;
          
          marker.action = visualization_msgs::Marker::ADD; 
          marker.pose.position.x = drop_x;
          marker.pose.position.y = drop_y;
          marker_pub.publish(marker);
        }
	break;
   case MOVING_TO_DROP_ZONE:
       // ROS_INFO("Drop (%f;%f)  (%f;%f)", current_x, current_y, drop_x, drop_y);
        
        if (isReachedGoal(drop_x, drop_y, current_x, current_y) && isRobotWait) {
	  ROS_INFO("Drop zone reached....");
          robotState = WAIT;
        }
	break;
   default: break;
 }

}

void timerCallback(const ros::TimerEvent&) {
  checkRobotPos = true;
}

void callbackOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (!checkRobotPos) return;
  checkRobotPos = false;

  double current_x = msg->pose.pose.position.x;
  double current_y = msg->pose.pose.position.y;
  double current_w = msg->pose.pose.orientation.w;

  double distance = sqrt(pow(last_x - current_x, 2) + pow(last_y - current_y, 2) + pow(last_w - current_w, 2));

  //ROS_INFO("distance %f ", distance);
  isRobotWait = distance <= 0.001; 

  process_robot_position(aml_x, aml_y);
  
  last_x = current_x;
  last_y = current_y;
  last_w = current_w;
}

void amlPosCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
    aml_x = msgAMCL->pose.pose.position.x;
    aml_y = msgAMCL->pose.pose.position.y;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("odom", 1000, callbackOdom);

  ros::Subscriber amlPosSub = n.subscribe("amcl_pose", 1000, amlPosCallback);  
  ros::Timer timer = n.createTimer(ros::Duration(0.2), timerCallback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pickup_x;
    marker.pose.position.y = pickup_y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    marker_pub.publish(marker);
    ROS_INFO("Place market"); 

    ros::spin();

    return 0;
}
