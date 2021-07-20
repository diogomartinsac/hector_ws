//#region [include]

#include "drone_middleware/WorldController.h"

//#endregion

namespace drone_middleware
{

  //#region [class]

  WorldController::WorldController(std::string nodeName, std::string pauseName, 
    std::string unpauseName, std::string restartName, std::string getSimInfoName,
    std::string multiStepAction, std::string worldStatsTopic, 
    std::string worldControlTopic, std::string physicsTopic, 
    unsigned int gazeboBufferSize, unsigned int gazeboTopicRate)
  {

    this->currentSim.startTime = ros::WallTime::now();
    this->currentSim.simTime = ros::Time(0);
    this->currentSim.pauseTime = ros::Time(0);
    this->currentSim.realTime = this->currentSim.startTime;
    this->currentSim.paused = false;
    this->currentSim.iterations = 0;
    this->currentSim.stepping = false;

    ros::param::set("/drone_middleware/sim_stepping", "false");

    //anuncio dos servicos
    //"gazebo_world_controller/pause"
    //"gazebo_world_controller/unpause"
    //"gazebo_world_controller/restart"
    //"gazebo_world_controller/get_sim_info"
    this->pauseService = this->n.advertiseService(nodeName + "/" + pauseName, 
      &WorldController::pauseCallback, this);
    this->unpauseService = this->n.advertiseService(nodeName + "/" + unpauseName, 
      &WorldController::unpauseCallback, this);
    this->restartService = this->n.advertiseService(nodeName + "/" + restartName, 
      &WorldController::restartCallback, this);

    this->getSimInfoService = this->n.advertiseService(nodeName + "/" + getSimInfoName, 
      &WorldController::getSimInfoCallback, this);

    //construindo action server e inicializando
    //"gazebo_world_controller/multi_step"
    /* this->multiStepServer = new actionlib::SimpleActionServer<drone_middleware::MultiStepAction>(this->n, 
    nodeName + "/" + multiStepAction, boost::bind(&WorldController::multiStepCallback, this, _1), false); */
    this->multiStepServer = new actionlib::SimpleActionServer<drone_middleware::MultiStepAction>(
      this->n, nodeName + "/" + multiStepAction, false);
    this->multiStepServer->start();

    //criacao e inicializacao do no do gazebo
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    this->node = node;
    this->node->Init();

    //inscrevendo no topico ~/world_stats
    this->statSubscriber = this->node->Subscribe(worldStatsTopic, &WorldController::worldStatsCallback, this);

    //criando um publicador para o topico gazebo ~/world_control
    //10, 1000
    this->controlPublisher = this->node->Advertise<gazebo::msgs::WorldControl>(
      worldControlTopic, gazeboBufferSize, gazeboTopicRate);
    
    //criando um publicador para o topico gazebo ~/physics
    this->physicsPublisher = this->node->Advertise<gazebo::msgs::Physics>(
      physicsTopic, gazeboBufferSize, gazeboTopicRate);

  }

  //#endregion

  //#region [callback]

  //callback do gazebo ~/world_stats
  void WorldController::worldStatsCallback(ConstWorldStatisticsPtr &_msg)
  { 
    this->currentSim.simTime.sec = _msg->sim_time().sec();
    this->currentSim.simTime.nsec = _msg->sim_time().nsec();
    this->currentSim.pauseTime.sec = _msg->pause_time().sec();
    this->currentSim.pauseTime.nsec = _msg->pause_time().nsec();
    this->currentSim.realTime = ros::WallTime::now();
    this->currentSim.paused = _msg->paused();
    this->currentSim.iterations = _msg->iterations();

    if(this->multiStepServer->isNewGoalAvailable())
    {

      //se ja havia uma operacao em curso, aborte-a
      if(this->multiStepServer->isActive())
      {
        drone_middleware::MultiStepResult resultMsg;
        //resultMsg.completed = this->currentSim.iterations - this->currentStep.stepIterations;
        resultMsg.completed = this->currentSim.iterations;
        this->multiStepServer->setPreempted(resultMsg);
      }

      const actionlib::SimpleActionServer<drone_middleware::MultiStepAction>::GoalConstPtr& 
        goal = this->multiStepServer->acceptNewGoal();

      //this->currentSim.iterationGoal = this->currentSim.iterations + goal->number;
      this->currentSim.iterationGoal = goal->number;

      this->currentStep.stepIterations = this->currentSim.iterations;
      this->currentStep.stepTime = this->currentSim.simTime;
      this->currentStep.stepTimeReal = ros::WallTime::now();

      this->currentSim.stepping = true;

      ros::param::set("/drone_middleware/sim_stepping", "true");
      
      //this->multiStep(goal->number);
      this->multiStep(goal->number - this->currentSim.iterations);

    }

    if(this->currentSim.stepping)
    {

      if(this->currentSim.iterations == this->currentSim.iterationGoal)
      {

        this->currentSim.stepping = false;

        ros::param::set("/drone_middleware/sim_stepping", "false");

        drone_middleware::MultiStepResult resultMsg;
        //resultMsg.completed = this->currentSim.iterations - this->currentStep.stepIterations;
        resultMsg.completed = this->currentSim.iterations;
        this->multiStepServer->setSucceeded(resultMsg);

        ROS_INFO("[DONE] Done. Sending response.");

        ROS_INFO("\n[STEP]\tSim time:\t%g\n"
          "[STEP]\tReal Time:\t%g\n"
          "[STEP]\tIterations:\t%lu", 
          (this->currentSim.simTime - this->currentStep.stepTime).toSec(), 
          (ros::WallTime::now() - this->currentStep.stepTimeReal).toSec(), 
          this->currentSim.iterations - this->currentStep.stepIterations);

      }
      else
      {
        #ifdef DISPLAY_ROS_INFO
        ROS_INFO("\n[SIM]\tSim time:\t%g\n"
          "[SIM]\tPause Time:\t%g\n"
          "[SIM]\tReal Time:\t%g\n"
          "%s\n"
          "[SIM]\tIterations:\t%lu\n",
          this->currentSim.simTime.toSec(), this->currentSim.pauseTime.toSec(), 
          (this->currentSim.realTime - this->currentSim.startTime).toSec(), 
          (this->currentSim.paused ? ("[SIM]\tPaused:\t\ttrue") : ("[SIM]\tPaused:\t\tfalse") ), 
          this->currentSim.iterations);
        #endif

        //possivel feedback estaria aqui
      }
    }
  }

  //callback de pausar
  bool WorldController::pauseCallback(drone_middleware::Pause::Request  &req,
    drone_middleware::Pause::Response &res)
  {
    
    this->pause();
    
    //indicar sucesso no servico
    //res.success = 1;
    return true;
  }

  //callback de despausar
  bool WorldController::unpauseCallback(drone_middleware::Unpause::Request  &req,
    drone_middleware::Unpause::Response &res)
  {
    
    this->unpause();
    
    //indicar sucesso no servico
    //res.success = 1;
    return true;
  }

  /*
  //callback de incrementar o tempo com passos discretos
  void WorldController::multiStepCallback(
    const actionlib::SimpleActionServer<drone_middleware::MultiStepAction>::GoalConstPtr& goal)
  {

    this->currentSim.iterationGoal = this->currentSim.iterations + goal->number;

    this->currentStep.stepIterations = this->currentSim.iterations;
    this->currentStep.stepTime = this->currentSim.simTime;
    this->currentStep.stepTimeReal = ros::WallTime::now();

    this->currentSim.stepping = true;
    
    this->multiStep(goal->number);
  }
  */

  //callback de reinicio
  bool WorldController::restartCallback(drone_middleware::Pause::Request  &req,
    drone_middleware::Pause::Response &res)
  {

    this->restart();
    
    //indicar sucesso no servico
    //res.success = 1;
    return true;
  }

  //callback de fornecer informações
  bool WorldController::getSimInfoCallback(drone_middleware::GetSimInfo::Request  &req,
    drone_middleware::GetSimInfo::Response &res)
  {
    
    //preencher a resposta
    res.startTime.sec = currentSim.startTime.sec;
    res.startTime.nsec = currentSim.startTime.nsec;
    res.realTime.sec = currentSim.realTime.sec;
    res.startTime.nsec = currentSim.startTime.nsec;
    res.simTime = currentSim.simTime;
    res.pauseTime = currentSim.pauseTime;
    res.iterations = currentSim.iterations;
    res.iterationGoal = currentSim.iterationGoal;
    res.paused = (unsigned int) currentSim.paused;
    res.stepping = (unsigned int) currentSim.stepping;
    return true;
  }

  //#endregion

  //#region [func]

  void WorldController::waitConnectionOnPublishers()
  {

    //esperar conexao
    this->controlPublisher->WaitForConnection();
    this->physicsPublisher->WaitForConnection();

  }

  void WorldController::pause()
  {

    //criando mensagem de controle
    gazebo::msgs::WorldControl controlMsg;
    //colocando pause para true
    controlMsg.set_pause(true);
    //envio

    controlPublisher->Publish(controlMsg, true);

    ros::WallRate pubRate(1000);
    while(!this->currentSim.paused) //BUSY WAITING
    {
      //ROS_DEBUG("%s\n", this->currentSim.paused ? "paused" : "not paused"); //DEBUG
      controlPublisher->Publish(controlMsg, true);
      pubRate.sleep();
    }

    ROS_INFO("[PAUSE] Gazebo was paused.");

  }

  void WorldController::unpause()
  {
    
    //criando mensagem de controle
    gazebo::msgs::WorldControl controlMsg;
    //colocando pause para false
    controlMsg.set_pause(false);
    //envio

    controlPublisher->Publish(controlMsg, true);

    ros::WallRate pubRate(1000);
    while(this->currentSim.paused) //BUSY WAITING
    {
      //ROS_DEBUG("%s\n", this->currentSim.paused ? "paused" : "not paused"); //DEBUG
      controlPublisher->Publish(controlMsg, true);
      pubRate.sleep();
    }

    ROS_INFO("[UNPAUSE] Gazebo was unpaused.");

  }

  //TODO: nao esta dando restart no gazebo no inicio

  void WorldController::restart()
  {

    //criando mensagem de controle
    gazebo::msgs::WorldControl controlMsg;
    //colocando pause para true
    controlMsg.set_pause(true);
    //colocando reset para true
    controlMsg.mutable_reset()->set_all(true);
    //envio

    controlPublisher->Publish(controlMsg, true);

    ros::WallRate pubRate(1000);
    while(this->currentSim.iterations != 0) //BUSY WAITING
    {
      //ROS_DEBUG("%i iterations\n", (int) this->currentSim.iterations); //DEBUG
      controlPublisher->Publish(controlMsg, true);
      pubRate.sleep();
    }

    ROS_INFO("[RESTART] Gazebo was restarted.");

  }

  //da passos de simulacao no gazebo
  //talvez seja bom mudar esse unsigned int steps para outro tipo
  void WorldController::multiStep(uint32_t steps)
  {

    //criando mensagem de controle
    gazebo::msgs::WorldControl controlMsg;
    //colocando numero de passos
    controlMsg.set_multi_step(steps);
    //envio
    controlPublisher->Publish(controlMsg, true);

    ROS_INFO("[STEP] Gazebo was told to step by %u steps.", steps);

  }

  //se rate = 0, gazebo simula tao rapido quanto possivel
  void WorldController::setRealTimeUpdateRate(float rate)
  {

    //criando mensagem de fisica
    gazebo::msgs::Physics physicsMsg;
    //ajustando parametro para que simule tao rapido quanto possivel se = 0
    //ou outra taxa informada
    physicsMsg.set_real_time_update_rate(rate);
    //envio
    physicsPublisher->Publish(physicsMsg, true);

    ROS_INFO("[PHYSICS] Gazebo physics real_time_update_rate was set to (%f).", rate);

  }

  //#endregion

}