//#region [include]

#include "drone_middleware/TeleoperatorText.h"

//#endregion

namespace drone_middleware
{

  //#region [func]

  void TeleoperatorText::displayUsageInfo()
  {

    //forma de uso: <numero_do_drone> [inicio_em_segundos] [x] [y] [z] [row] [pitch] [yaw]
    ROS_INFO("[USAGE] <drone_number> [start_time_in_seconds] [x] [y] [z] [row] [pitch] [yaw]");

    ROS_INFO("[START] Press enter to start.");

  }

  bool TeleoperatorText::getOneInput(drone_middleware::ScheduleMovement &srv_msg)
  {
    float start_at;

    do
    {
      std::cin.clear(); // reset failbit
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); //skip bad input
    
      //entrar comando
      ROS_INFO("[INPUT] Enter command: ");
    
      //pegar entrada ate que seja correta
      std::cin >> srv_msg.request.droneNumber >> 
      start_at >> 
      srv_msg.request.movement.linear.x >> 
      srv_msg.request.movement.linear.y >> 
      srv_msg.request.movement.linear.z >> 
      srv_msg.request.movement.angular.x >> 
      srv_msg.request.movement.angular.y >> 
      srv_msg.request.movement.angular.z;

    } while(std::cin.fail());
  
    srv_msg.request.startAt.sec = ros::Time(start_at).sec;
    srv_msg.request.startAt.nsec = ros::Time(start_at).nsec;

    return true;

  }

  int TeleoperatorText::enterLoop()
  {

    drone_middleware::ScheduleMovement srv_msg;

    this->displayUsageInfo();

    while (ros::ok())
    {
      
      this->getOneInput(srv_msg);
      this->callSchedulerServer(srv_msg);

      ros::spinOnce();

    }

    return 0;
  }

  //#endregion

}