//#region [include]

#include "drone_middleware/CoreWaypoint.h"

//#endregion

//#region [main]

void debugFunction(drone_middleware::CoreWaypoint * waypointCore)
{
  static uint32_t seq = 0;

  //ROS_DEBUG("[DBG] Taking off drone0.");
  drone_middleware::DroneWaypoint waypoint;
  waypoint.droneNumber = 0;
  //seq++;
  waypoint.seqNum = seq;
  waypoint.takeoff = 1;
  waypoint.land = 0;
  waypoint.landAt = 0;
  waypointCore->scheduleWaypoint(waypoint);
  //waypointCore->scheduleWaypoint(waypoint);
  //waypointCore->scheduleWaypoint(waypoint);

  //seq++;
  waypoint.seqNum = seq;
  waypoint.pose.position.x = 0;
  waypoint.pose.position.y = 0;
  waypoint.pose.position.z = 3;
  waypoint.pose.orientation.x = 0;
  waypoint.pose.orientation.y = 0;
  waypoint.pose.orientation.z = 0;
  waypoint.pose.orientation.w = 0;
  waypoint.takeoff = 0;
  //ROS_DEBUG("[DBG] Waypoint for drone0: x: %.1f, y: %.1f, z:%.1f.", waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
  waypointCore->scheduleWaypoint(waypoint);

  int i, j;
  for(i = 0; i < 6; i++)
  {
    for(j = 0; j < 6; j++)
    {
      //seq++;
      waypoint.seqNum = seq;
      waypoint.pose.position.x = i*2 - 6;
      waypoint.pose.position.y = j*2 - 6;
      waypoint.pose.position.z = 3;
      //ROS_DEBUG("[DBG] Waypoint for drone0: x: %.1f, y: %.1f, z:%.1f.", waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
      waypointCore->scheduleWaypoint(waypoint);
    }
  }

  /*
  waypoint.pose.position.z = 2.5;
  ROS_DEBUG("[DBG] Waypoint for drone0: x: %.1f, y: %.1f, z:%.1f.", waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
  seq++;
  waypoint.seqNum = seq;
  waypointCore->scheduleWaypoint(waypoint);
  waypoint.pose.position.z = 2;
  ROS_DEBUG("[DBG] Waypoint for drone0: x: %.1f, y: %.1f, z:%.1f.", waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
  seq++;
  waypoint.seqNum = seq;
  waypointCore->scheduleWaypoint(waypoint);
  waypoint.pose.position.z = 1.5;
  ROS_DEBUG("[DBG] Waypoint for drone0: x: %.1f, y: %.1f, z:%.1f.", waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
  seq++;
  waypoint.seqNum = seq;
  waypointCore->scheduleWaypoint(waypoint);
  waypoint.pose.position.z = 1;
  ROS_DEBUG("[DBG] Waypoint for drone0: x: %.1f, y: %.1f, z:%.1f.", waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
  seq++;
  waypoint.seqNum = seq;
  waypointCore->scheduleWaypoint(waypoint);
  waypoint.pose.position.z = 0.5;
  ROS_DEBUG("[DBG] Waypoint for drone0: x: %.1f, y: %.1f, z:%.1f.", waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
  seq++;
  waypoint.seqNum = seq;
  waypointCore->scheduleWaypoint(waypoint);
  */

  //waypointCore->clearWaypointQueue(1, 0);

  waypoint.land = 1;
  //ROS_DEBUG("[DBG] Landing drone0.");
  //seq++;
  waypoint.seqNum = seq;
  waypointCore->scheduleWaypoint(waypoint);
}

int main(int argc, char **argv)
{

  namespace dm = drone_middleware;

  // boilerplate necessario do ros
  ros::init(argc, argv, "drone_waypoint_core");

  if(dm::CoreWaypoint::setAndGetNDrones("/drone_middleware/n_drones") == -1)
    ros::shutdown();

  dm::CoreWaypoint * waypointCore = 
    new dm::CoreWaypoint("drone", "action/takeoff", "action/pose", "action/landing", 
    "command/twist_limit", "drone_waypoint_core", "waypoint_state", "schedule_waypoint", 
    "cancel_waypoints", "get_current_goal", "set_maximum_attempts", "set_twist_limit", 100, 3);

  waypointCore->waitForActionServers();

  ROS_INFO("[READY] Hector action servers are ready.");

  geometry_msgs::Twist twistLimit;
  twistLimit.linear.x = 1;
  twistLimit.linear.y = 1;
  twistLimit.linear.z = 1;
  twistLimit.angular.x = 1;
  twistLimit.angular.y = 1;
  twistLimit.angular.z = 1;
  waypointCore->sendTwistLimit(1, 0, twistLimit);

  int NODES_ONLINE;
  if(!ros::param::get("/drone_middleware/nodes_online", NODES_ONLINE)) NODES_ONLINE = 0;
  ros::param::set("/drone_middleware/nodes_online", NODES_ONLINE+1);

  ROS_INFO("[READY] drone_waypoint_core is ready.");

  //DEBUG
  //debugFunction(waypointCore);
  
  //loop do ROS
  ros::spin();

  return 0;

}

//#endregion

