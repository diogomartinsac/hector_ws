//#region [include]

#include "drone_middleware/Teleoperator.h"

//#endregion

namespace drone_middleware
{

  //#region [class]

  Teleoperator::Teleoperator(std::string serverName, std::string serviceName)
  {

    //criando cliente do servico de escalonar movimentos
    this->movementSchedulerClient = this->n.serviceClient<drone_middleware::ScheduleMovement>(serverName + "/" + serviceName);

  }

  //#endregion

  //#region [func]

  int Teleoperator::callSchedulerServer(drone_middleware::ScheduleMovement srv_msg)
  {

    //chamando servico
    this->movementSchedulerClient.call(srv_msg);
    //verificando
    if(!srv_msg.response.success) ROS_WARN("[SCHEDULE_MOVEMENT] A movement scheduling was not successful.");
    else
    {
      #ifdef DISPLAY_ROS_INFO
      ROS_INFO("[REQ] [%gs] [drone: %i] [%gx, %gy, %gz] [%gx, %gy, %gz]", 
        srv_msg.request.startAt.toSec(), 
        srv_msg.request.droneNumber, 
        srv_msg.request.movement.linear.x, 
        srv_msg.request.movement.linear.y, 
        srv_msg.request.movement.linear.z, 
        srv_msg.request.movement.angular.x, 
        srv_msg.request.movement.angular.y, 
        srv_msg.request.movement.angular.z);
      #endif

    }

    return srv_msg.response.success;

  }

  //#endregion
}