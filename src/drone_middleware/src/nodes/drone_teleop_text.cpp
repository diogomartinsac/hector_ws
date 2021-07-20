//#region [include]

#include "drone_middleware/TeleoperatorText.h"

//#endregion

//#region [main]

int main(int argc, char **argv)
{
  namespace dm = drone_middleware;

  // boilerplate necessario do ros
  ros::init(argc, argv, "drone_teleop_text");

  dm::TeleoperatorText * teleopText = 
    new dm::TeleoperatorText("drone_teleop_core", "schedule_movement");

  return teleopText->enterLoop();
}

//#endregion