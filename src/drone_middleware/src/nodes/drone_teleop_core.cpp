//#region [include]

#include "drone_middleware/CoreTeleoperator.h"

//#endregion

//#region [main]

int main(int argc, char **argv)
{

  namespace dm = drone_middleware;

  //boilerplate necessario do ros
  ros::init(argc, argv, "drone_teleop_core");

  //pegando parametro do numero de drones
  if(dm::CoreTeleoperator::setAndGetNDrones("/drone_middleware/n_drones") == -1)
    ros::shutdown();

  //dm::CoreTeleoperator::resetDroneStates();

  dm::CoreTeleoperator * teleopCore = 
    new dm::CoreTeleoperator("drone_teleop_core", "schedule_movement", 
    "clock", "drone", "cmd_vel", "enable_motors", 100);

  teleopCore->startDroneMotors();

  teleopCore->startPublishing();

  int NODES_ONLINE;
  if(!ros::param::get("/drone_middleware/nodes_online", NODES_ONLINE)) NODES_ONLINE = 0;
  ros::param::set("/drone_middleware/nodes_online", NODES_ONLINE+1);

  ROS_INFO("[READY] drone_teleop_core is ready.");
  
  //loop do ROS
  ros::spin();

  return 0;

}

//#endregion

