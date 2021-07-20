//#region [include]

#include "drone_middleware/CoreTeleoperator.h"

//#endregion

namespace drone_middleware
{

  //#region [class]

  //numero de drones, passado como parametro ROS
  int CoreTeleoperator::N_DRONES;

  /*
  //vetor de movimentos atualmente sendo enviados aos drones
  std::vector<drone_middleware::DroneMovement> CoreTeleoperator::droneStates;

  //fila ordenada de movimentos
  std::priority_queue<drone_middleware::DroneMovement, 
    std::vector<drone_middleware::DroneMovement>, 
    std::greater<drone_middleware::DroneMovement>> CoreTeleoperator::movements;
  */

  //construtor
  CoreTeleoperator::CoreTeleoperator(std::string nodeName, std::string serviceName, 
    std::string clockTopic, std::string prefix, std::string topicSuffix, 
    std::string serviceSuffix, unsigned int bufferSize)
  {

    this->publishing = false;

    this->resetDroneStates();

    //"drone_teleop_core/schedule_movement"
    scheduleMovementService = n.advertiseService(nodeName + "/" + serviceName, 
      &CoreTeleoperator::scheduleMovementCallback, this);

    /*dronePublishers.push_back(n.advertise<geometry_msgs::Twist>("drone0/cmd_vel", 10));
    dronePublishers.push_back(n.advertise<geometry_msgs::Twist>("drone1/cmd_vel", 10));
    dronePublishers.push_back(n.advertise<geometry_msgs::Twist>("drone2/cmd_vel", 10));
    dronePublishers.push_back(n.advertise<geometry_msgs::Twist>("drone3/cmd_vel", 10));
    dronePublishers.push_back(n.advertise<geometry_msgs::Twist>("drone4/cmd_vel", 10));*/
    

    //loop para realizar acoes para todos os drones
    std::stringstream ss;
    for(int i = 0; i < N_DRONES; i++)
    {
      //limpar o stream e colocar nele o número atual do loop
      ss.str(std::string());
      ss << i;

      //TODO: mensagens cmd_vel só estão sendo enviadas para um único drone, o drone zero
      //5 publishers são criados mas só o do 0 funciona
    
      //anunciando os topicos de controle dos drones
      //"drone" "/cmd_vel"
      dronePublishers.push_back(n.advertise<geometry_msgs::Twist>(prefix + ss.str() + 
        "/" + topicSuffix, bufferSize));

      //ROS_DEBUG("%i %s", i, dronePublishers[i].getTopic().c_str()); //DEBUG
      
      //criando os clientes do servico de iniciar motor
      //"drone" "/enable_motors"
      motorStarterClients.push_back(n.serviceClient<hector_uav_msgs::EnableMotors>(prefix + 
        ss.str() + "/" + serviceSuffix));

    }

    //inscricao no topico
    //"clock"
    // !!! forma de chamar subscribe com uso de member function: !!!
    clockSubscriber = n.subscribe(clockTopic, bufferSize, &CoreTeleoperator::clockCallback, this);

    //ROS_DEBUG("Quantidade de publishers = %lu", dronePublishers.size()); //DEBUG

  }

  //#endregion

  //#region [callback]

  void CoreTeleoperator::clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg)
  {

    //checando se tempo atual >= que desejado e se fila vazia
    while((this->movements.size() > 0) && (msg->clock.sec >= this->movements.top().startAt.sec) && (msg->clock.nsec >= this->movements.top().startAt.nsec))
    {
      this->droneStates[this->movements.top().droneNumber].startAt.sec = this->movements.top().startAt.sec;
      this->droneStates[this->movements.top().droneNumber].startAt.nsec = this->movements.top().startAt.nsec;
      this->droneStates[this->movements.top().droneNumber].movement.linear.x = this->movements.top().movement.linear.x;
      this->droneStates[this->movements.top().droneNumber].movement.linear.y = this->movements.top().movement.linear.y;
      this->droneStates[this->movements.top().droneNumber].movement.linear.z = this->movements.top().movement.linear.z;
      this->droneStates[this->movements.top().droneNumber].movement.angular.x = this->movements.top().movement.angular.x;
      this->droneStates[this->movements.top().droneNumber].movement.angular.y = this->movements.top().movement.angular.y;
      this->droneStates[this->movements.top().droneNumber].movement.angular.z = this->movements.top().movement.angular.z;
      
      //excluindo primeiro elemento da fila
      this->movements.pop();
    }
    
    if(this->publishing)
    {

      for(int i = 0; i < N_DRONES; i++)
      { 
        //publicando
        dronePublishers[i].publish(this->droneStates[i].movement);
        //ROS_DEBUG("drone%i: %g %g %g %g %g %g", i, droneStates[i].movement.linear.x, droneStates[i].movement.linear.y, 
        //droneStates[i].movement.linear.z, droneStates[i].movement.angular.x, 
        //droneStates[i].movement.angular.y, droneStates[i].movement.angular.z); //DEBUG

        #ifdef DISPLAY_ROS_INFO
        ROS_INFO("[CMD_VEL] [%gs] [drone: %i] [%gx, %gy, %gz] [%gx, %gy, %gz]", 
          this->droneStates[i].startAt.toSec(), 
          this->droneStates[i].droneNumber, 
          this->droneStates[i].movement.linear.x, 
          this->droneStates[i].movement.linear.y, 
          this->droneStates[i].movement.linear.z, 
          this->droneStates[i].movement.angular.x, 
          this->droneStates[i].movement.angular.y, 
          this->droneStates[i].movement.angular.z);
        #endif
      }

    }

  }

  //callback de inserir movimento na fila
  bool CoreTeleoperator::scheduleMovementCallback(drone_middleware::ScheduleMovement::Request  &req, 
    drone_middleware::ScheduleMovement::Response &res)
  {
    ros::Time current_time = ros::Time::now();
    
    if((current_time <= req.startAt) || (req.startAt.isZero()))
    {
      if(req.droneNumber > N_DRONES-1)
      {
        ROS_WARN("[WARNING] Tried to add command for drone that does not exist!");
        res.success = 0;
        return false;
      }
      
      #ifdef DISPLAY_ROS_INFO
      ROS_INFO("[SCHEDULE_MOVEMENT] [%gs] [drone: %i] [%gx, %gy, %gz] [%gx, %gy, %gz]", 
        req.startAt.toSec(), 
        req.droneNumber, 
        req.movement.linear.x, 
        req.movement.linear.y, 
        req.movement.linear.z, 
        req.movement.angular.x, 
        req.movement.angular.y, 
        req.movement.angular.z);
      #endif
      
      drone_middleware::DroneMovement newMovement;
      newMovement.movement = req.movement;
      newMovement.droneNumber = req.droneNumber;
      newMovement.startAt = req.startAt;
      this->movements.push(newMovement);
      res.success = 1;
      return true; 
    }
    else
    {
      ROS_WARN("[WARNING] Tried to add movement with scheduled time lesser than current time!");
      res.success = 0;
      return false;
    }
    
  }

  //#endregion

  //#region [func]

  //chamar no inicio antes de nodeInit para configurar numero de drones 
  //a partir do servidor de parametros ros
  int CoreTeleoperator::setAndGetNDrones(std::string paramName)
  {
    //pegando parametro do numero de drones
    if(!ros::param::get(paramName, N_DRONES))
    {
      ROS_ERROR("[PARAM] Unable to get the ROS parameter %s", paramName.c_str());
      return -1;
    }
    else return N_DRONES;
  }

  //limpar e inicializar dronestates
  void CoreTeleoperator::resetDroneStates()
  {

    this->droneStates.clear();

    //loop para realizar acoes para todos os drones
    for(int i = 0; i < N_DRONES; i++)
    {   
      this->droneStates.push_back(drone_middleware::DroneMovement());
      
      //preencher estados iniciais de movimento dos drones
      this->droneStates[i].startAt.sec = 0;
      this->droneStates[i].startAt.nsec = 0;
      this->droneStates[i].droneNumber = i; // numero do drone
      this->droneStates[i].movement.linear.x = 0;
      this->droneStates[i].movement.linear.y = 0;
      this->droneStates[i].movement.linear.z = 0;
      this->droneStates[i].movement.angular.x = 0;
      this->droneStates[i].movement.angular.y = 0;
      this->droneStates[i].movement.angular.z = 0;

    }

  }

  void CoreTeleoperator::startDroneMotors()
  {
  
    //criar mensagem do servico de iniciar motor
    hector_uav_msgs::EnableMotors srv;
    //campo de iniciar motor = true
    srv.request.enable = true;
    
    //loop para realizar acoes para todos os drones
    for(int i = 0; i < N_DRONES; i++)
    {     

      while(!this->motorStarterClients[i].waitForExistence(ros::Duration(5.0)))
      {
        ROS_WARN("[SERVICE] Motor enable service not found");
      }

      do
      { 

        //chamando servico de iniciar motor ate obter sucesso
        this->motorStarterClients[i].call(srv);
        
      }
      while(!srv.response.success);

    }

    ROS_INFO("[READY] Drone motors started.");

  }

  void CoreTeleoperator::startPublishing() 
  {

    ROS_INFO("[START] Now publishing to drones according to clock.");

    this->publishing = true; 
  }

  void CoreTeleoperator::stopPublishing()
  {

    ROS_INFO("[STOP] Now discarding commands according to clock.");

    this->publishing = false;
  }

  //#endregion

}