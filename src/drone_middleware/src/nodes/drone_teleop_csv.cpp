//#region [include]

#include "drone_middleware/TeleoperatorCSV.h"

//#endregion

//#region [main]

int main(int argc, char **argv)
{

  namespace dm = drone_middleware;
  
  // boilerplate necessario do ros
  ros::init(argc, argv, "drone_teleop_csv");

  dm::TeleoperatorCSV * teleopCSV = 
    new dm::TeleoperatorCSV("drone_teleop_core", "schedule_movement", 
    dm::TeleoperatorCSV::getPath(argc, argv, "/drone_middleware/csv_path"));

  teleopCSV->enterLoop();

  int NODES_ONLINE;
  //pegando parametro do numero de drones
  if(!ros::param::get("/drone_middleware/nodes_online", NODES_ONLINE)) NODES_ONLINE = 0;
  ros::param::set("/drone_middleware/nodes_online", NODES_ONLINE+1);

  return 0;
}

//#endregion