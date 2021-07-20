//#region [include]

#include "drone_middleware/CoreWaypoint.h"

//#endregion

namespace drone_middleware
{

  //#region [class]

  //numero de drones, passado como parametro ROS
  int CoreWaypoint::N_DRONES;

  //construtor
  CoreWaypoint::CoreWaypoint(std::string prefix, std::string takeoffSuffix, std::string poseSuffix, 
    std::string landingSuffix, std::string limitSuffix, std::string nodeName, 
    std::string completedWaypointsTopic, std::string scheduleWaypointService, std::string cancelWaypointsService, 
    std::string getCurrentGoalService, std::string setMaximumAttemptsService, 
    std::string setTwistLimitService, uint16_t bufferSize, uint8_t maximumFailedAttempts)
  {

    //this->clearWaypointQueue(true, 0);
    std::set< drone_middleware::DroneWaypoint,  
      std::less< drone_middleware::DroneWaypoint > > emptyQueue;

    this->prefix = prefix;

    this->maximumFailedAttempts = maximumFailedAttempts;

    //drone_waypoint_core/completed_waypoints
    this->WaypointStatePublisher = this->n.advertise<drone_middleware::WaypointState>(
      nodeName + "/" + completedWaypointsTopic, bufferSize);

    //criando os servidores da classe
    this->scheduleWaypointServer = this->n.advertiseService(nodeName + "/" + scheduleWaypointService, 
      &CoreWaypoint::scheduleWaypointCallback, this);

    this->cancelWaypointsServer = this->n.advertiseService(nodeName + "/" + cancelWaypointsService, 
      &CoreWaypoint::cancelWaypointsCallback, this);
    
    this->getCurrentGoalServer = this->n.advertiseService(nodeName + "/" + getCurrentGoalService, 
      &CoreWaypoint::getCurrentGoalCallback, this);
    
    this->setMaximumAttemptsServer = this->n.advertiseService(nodeName + "/" + setMaximumAttemptsService, 
      &CoreWaypoint::setMaximumAttemptsCallback, this);
    
    this->setTwistLimitServer = this->n.advertiseService(nodeName + "/" + setTwistLimitService, 
      &CoreWaypoint::setTwistLimitCallback, this);

    //loop para realizar acoes para todos os drones
    std::stringstream ss;
    for(int droneNumber = 0; droneNumber < N_DRONES; droneNumber++)
    {
      //limpar o stream e colocar nele o número atual do loop
      ss.str(std::string());
      ss << droneNumber;

      //enchendo o vetor de filas vazias
      this->waypoints.push_back(emptyQueue);
    
      //criando os clientes de ações
      //"drone" "0" "/" "action/takeoff"
      this->takeoffClients.push_back( new actionlib::SimpleActionClient<hector_uav_msgs::TakeoffAction>(
        this->n, prefix + ss.str() + "/" + takeoffSuffix, true) );

      //"drone" "0" "/" "action/pose"
      this->poseClients.push_back( new actionlib::SimpleActionClient<hector_uav_msgs::PoseAction>(
        this->n, prefix + ss.str() + "/" + poseSuffix, true) );

      //"drone" "0" "/" "action/landing"
      this->landingClients.push_back( new actionlib::SimpleActionClient<hector_uav_msgs::LandingAction>(
        this->n, prefix + ss.str() + "/" + landingSuffix, true) );

      //preenchendo os objetivos atuais como objetivos vazios
      this->currentGoals.push_back(drone_middleware::DroneWaypoint());
      this->currentGoals[droneNumber].seqNum = 0;

      //definindo que nenhum dos clientes esta sendo usado ainda
      this->actionStates[TAKEOFF].push_back(SUCCEEDED);
      this->actionStates[POSE].push_back(SUCCEEDED);
      this->actionStates[LANDING].push_back(SUCCEEDED);

      //inicializando contadores de falhas
      this->failCounters.push_back(0);

      //criando publicadores de limite de velocidade
      twistLimitPublishers.push_back(this->n.advertise<geometry_msgs::Twist>(prefix + ss.str() + 
        "/" + limitSuffix, bufferSize));

    }

  }

  //#endregion

  //#region [callback]

  void CoreWaypoint::takeoffGoalCallback(const actionlib::SimpleClientGoalState& state,
    const hector_uav_msgs::TakeoffResult::ConstPtr& result)
  {
    for(int droneNumber = 0; droneNumber < N_DRONES; droneNumber++)
    {
      //O erro citado na documentação que está presente no github ocorreria aqui,
      //se a função getState() for chamada em um SimpleActionClient que não possui um objetivo em curso,
      //uma mensagem de erro é impressa em vermelho na tela.
      //Apesar dessa checagem ser motivada pelo erro, ela pode acabar melhorando o desempenho,
      //no caso em que houverem poucos objetivos simultâneos.

      //descobre qual cliente de ação deve ser usado
      if(this->actionStates[TAKEOFF][droneNumber] == ACTIVE && 
        this->takeoffClients[droneNumber]->getState() == state)
      {
        //executa se ação foi concluída com sucesso
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          //reseta contador de falhas e atualiza objetivo
          this->failCounters[droneNumber] = 0;
          this->removeCurrentGoalAndUpdate(droneNumber, SUCCEEDED);
        }
        else
        //executa se ação falhou
        {
          //incrementa e checa contador de falhas
          this->failCounters[droneNumber]++;
          if(this->failCounters[droneNumber] < this->maximumFailedAttempts)
          {
            //se não violou limite, executa de novo
            this->handleWaypoint(this->currentGoals[droneNumber]);
          }
          else
          //executa se violou limite
          {
            //reseta fail counter, atualiza estado
            this->failCounters[droneNumber] = 0;
            this->actionStates[TAKEOFF][droneNumber] = FAILED;
            ROS_WARN("[WARNING] Action failed too many times for drone%i.", droneNumber);

            //publica a falha no tópico
            this->WaypointStatePublisher.publish(
              this->createWaypointStateMsg(this->currentGoals[droneNumber], FAILED));             
          }
        }
      }
    }
  }

  void CoreWaypoint::takeoffActiveCallback(){}

  void CoreWaypoint::takeoffFeedbackCallback(
    const hector_uav_msgs::TakeoffFeedback::ConstPtr& feedback){}

  void CoreWaypoint::poseGoalCallback(const actionlib::SimpleClientGoalState& state,
    const hector_uav_msgs::PoseResult::ConstPtr& result)
  {
    for(int droneNumber = 0; droneNumber < N_DRONES; droneNumber++)
    {
      //O erro citado na documentação que está presente no github ocorreria aqui,
      //se a função getState() for chamada em um SimpleActionClient que não possui um objetivo em curso,
      //uma mensagem de erro é impressa em vermelho na tela.
      //Apesar dessa checagem ser motivada pelo erro, ela pode acabar melhorando o desempenho,
      //no caso em que houverem poucos objetivos simultâneos.

      //descobre qual cliente de ação deve ser usado
      if(this->actionStates[POSE][droneNumber] == ACTIVE && this->poseClients[droneNumber]->getState() == state)
      {
        //executa se ação foi concluída com sucesso
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          //reseta contador de falhas e atualiza objetivo
          this->failCounters[droneNumber] = 0;
          this->removeCurrentGoalAndUpdate(droneNumber, SUCCEEDED);
        }
        else
        //executa se ação falhou
        {
          //incrementa e checa contador de falhas
          this->failCounters[droneNumber]++;
          if(this->failCounters[droneNumber] < this->maximumFailedAttempts)
          {
            //se não violou limite, executa de novo
            this->handleWaypoint(this->currentGoals[droneNumber]);
          }
          else
          //executa se violou limite
          {
            //reseta fail counter, atualiza estado
            this->failCounters[droneNumber] = 0;
            this->actionStates[POSE][droneNumber] = FAILED;
            ROS_WARN("[WARNING] Action failed too many times for drone%i.", droneNumber);

            //publica a falha no tópico
            this->WaypointStatePublisher.publish(
              this->createWaypointStateMsg(this->currentGoals[droneNumber], FAILED));
          }
        }
      }
    }
  }

  void CoreWaypoint::poseActiveCallback(){}

  void CoreWaypoint::poseFeedbackCallback(
    const hector_uav_msgs::PoseFeedback::ConstPtr& feedback){}

  void CoreWaypoint::landingGoalCallback(const actionlib::SimpleClientGoalState& state,
    const hector_uav_msgs::LandingResult::ConstPtr& result)
  {
    for(int droneNumber = 0; droneNumber < N_DRONES; droneNumber++)
    {
      //O erro citado na documentação que está presente no github ocorreria aqui,
      //se a função getState() for chamada em um SimpleActionClient que não possui um objetivo em curso,
      //uma mensagem de erro é impressa em vermelho na tela.
      //Apesar dessa checagem ser motivada pelo erro, ela pode acabar melhorando o desempenho,
      //no caso em que houverem poucos objetivos simultâneos.

      //descobre qual cliente de ação deve ser usado
      if(this->actionStates[LANDING][droneNumber] == ACTIVE && this->landingClients[droneNumber]->getState() == state)
      {
        //executa se ação foi concluída com sucesso
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          //reseta contador de falhas e atualiza objetivo
          this->failCounters[droneNumber] = 0;
          this->removeCurrentGoalAndUpdate(droneNumber, SUCCEEDED);
        }
        else
        //executa se ação falhou
        {
          //incrementa e checa contador de falhas
          this->failCounters[droneNumber]++;
          if(this->failCounters[droneNumber] < this->maximumFailedAttempts)
          {
            //se não violou limite, executa de novo
            this->handleWaypoint(this->currentGoals[droneNumber]);
          }
          else
          //executa se violou limite
          {
            //reseta fail counter, atualiza estado
            this->failCounters[droneNumber] = 0;
            this->actionStates[LANDING][droneNumber] = FAILED;
            ROS_WARN("[WARNING] Action failed too many times for drone%i.", droneNumber);

            //publica a falha no tópico
            this->WaypointStatePublisher.publish(
              this->createWaypointStateMsg(this->currentGoals[droneNumber], FAILED));
          }
        }
      }
    }
  }

  void CoreWaypoint::landingActiveCallback(){}

  void CoreWaypoint::landingFeedbackCallback(
    const hector_uav_msgs::LandingFeedback::ConstPtr& feedback){}

  bool CoreWaypoint::scheduleWaypointCallback(
    drone_middleware::ScheduleWaypoint::Request  &req, 
    drone_middleware::ScheduleWaypoint::Response &res)
  {
    //cria objeto waypoint
    drone_middleware::DroneWaypoint waypoint;
    waypoint.droneNumber = req.droneNumber;
    waypoint.seqNum = req.seqNum;
    waypoint.pose = req.pose;
    waypoint.takeoff = req.takeoff;
    waypoint.land = req.land;
    waypoint.landAt = req.landAt;

    //envia na resposta
    res.success = this->scheduleWaypoint(waypoint);

    return true;
  }

  bool CoreWaypoint::cancelWaypointsCallback(
    drone_middleware::CancelWaypoints::Request  &req, 
    drone_middleware::CancelWaypoints::Response &res)
  {
    //inicializa contador
    res.numCanceled = 0;

    //chama funções de cancelamento
    if(req.clearQueue) 
      res.numCanceled += this->clearWaypointQueue(req.all, req.droneNumber);
    if(req.cancelCurrent) 
      res.numCanceled += this->cancelCurrentGoal(req.all, req.droneNumber);

    return true;
  }

  bool CoreWaypoint::getCurrentGoalCallback(
    drone_middleware::GetCurrentGoal::Request  &req, 
    drone_middleware::GetCurrentGoal::Response &res)
  {
    //cria objeto waypoint
    drone_middleware::DroneWaypoint waypoint = 
      this->currentGoals[req.droneNumber];

    //coloca na resposta
    res.droneNumber = waypoint.droneNumber;
    res.seqNum = waypoint.seqNum;
    res.pose = waypoint.pose;
    res.takeoff = waypoint.takeoff;
    res.land = waypoint.land;
    res.landAt = waypoint.landAt;

    //obtem estado do objetivo atual
    res.actionState = 
      this->actionStates[this->getActionType(waypoint)][req.droneNumber];

    return true;
  }

  bool CoreWaypoint::setMaximumAttemptsCallback(
    drone_middleware::SetMaximumAttempts::Request  &req, 
    drone_middleware::SetMaximumAttempts::Response &res)
  {
    //setter
    this->setMaximumAttempts(req.maximumAttempts);

    return true;
  }

  bool CoreWaypoint::setTwistLimitCallback(
    drone_middleware::SetTwistLimit::Request  &req, 
    drone_middleware::SetTwistLimit::Response &res)
  {
    //setter
    this->sendTwistLimit(req.all, req.droneNumber, req.limit);

    return true;
  }

  //#endregion

  //#region [func]

  //chamar no inicio antes de nodeInit para configurar numero de drones 
  //a partir do servidor de parametros ros
  int CoreWaypoint::setAndGetNDrones(std::string paramName)
  {
    //pegando parametro do numero de drones
    if(!ros::param::get(paramName, N_DRONES))
    {
      ROS_ERROR("[PARAM] Unable to get the ROS parameter %s", paramName.c_str());
      return -1;
    }
    else return N_DRONES;
  }

  uint16_t CoreWaypoint::clearWaypointQueue(uint8_t all, uint16_t droneNumber)
  {
    //cria objeto temporario
    std::set< drone_middleware::DroneWaypoint,  
      std::less< drone_middleware::DroneWaypoint > > tempQueue;

    //cria iterador
    std::set<drone_middleware::DroneWaypoint>::iterator it;

    //inicializa contador
    uint16_t returnValue = 0;

    //checa se ação é para todos os drones
    if(all)
    {
      for(droneNumber = 0; droneNumber < N_DRONES; droneNumber++)
      {
        //acrescenta tamanho da fila ao contador
        returnValue += this->waypoints[droneNumber].size();

        //itera sobre a estrutura
        for(it = this->waypoints[droneNumber].begin(); 
          it != this->waypoints[droneNumber].end();
          it = std::next(it))
        {
          //publica todos os comandos como cancelados no tópico
          this->WaypointStatePublisher.publish(
            this->createWaypointStateMsg(*it, CANCELED));
        }

        //substitui a estrutura por uma versão vazia
        this->waypoints[droneNumber].swap(tempQueue);
      }
    }
    else
    {
      if(droneNumber < N_DRONES)
      {
        //acrescenta tamanho da fila ao contador
        returnValue += this->waypoints[droneNumber].size();

        //itera sobre a estrutura
        for(it = this->waypoints[droneNumber].begin(); 
          it != this->waypoints[droneNumber].end();
          it = std::next(it))
        {
          //publica todos os comandos como cancelados no tópico
          this->WaypointStatePublisher.publish(
            this->createWaypointStateMsg(*it, CANCELED));
        }

        //substitui a estrutura por uma versão vazia
        this->waypoints[droneNumber].swap(tempQueue);
      }
      //executa se droneNumber inválido é informado
      else ROS_WARN("[WARNING] Tried to clear waypoint queue of nonexistent drone!");
    }

    return returnValue;
  }

  void CoreWaypoint::waitForActionServers()
  {
    //aguarda que todos os servidores de ação do hector_quadrotor estejam prontos
    for(int droneNumber = 0; droneNumber < N_DRONES; droneNumber++)
    {
      this->takeoffClients[droneNumber]->waitForServer();
      this->poseClients[droneNumber]->waitForServer();
      this->landingClients[droneNumber]->waitForServer();
    }
  }

  uint16_t CoreWaypoint::cancelCurrentGoal(uint8_t all, uint16_t droneNumber)
  {
    //inicializa contador
    uint16_t returnValue = 0;

    //checa se deve ser executado para todos os drones
    if(all)
    {
      for(int droneNumber = 0; droneNumber < N_DRONES; droneNumber++)
      {
        //checa se é possível cancelar
        //objetivos já cancelados ou completados com sucesso não podem ser cancelados
        if(!this->isAbleToReceiveActions(droneNumber))
        {
        //incrementa contador, cancela objetivos no servidor e publica no tópico
        returnValue++;
        this->takeoffClients[droneNumber]->cancelAllGoals();
        this->poseClients[droneNumber]->cancelAllGoals();
        this->landingClients[droneNumber]->cancelAllGoals();
        this->removeCurrentGoalAndUpdate(droneNumber, CANCELED);
        }
      }
    }
    else
    {
      if(droneNumber < N_DRONES)
      {
        //checa se é possível cancelar
        //objetivos já cancelados ou completados com sucesso não podem ser cancelados
        if(!this->isAbleToReceiveActions(droneNumber)) 
        {
        //incrementa contador, cancela objetivos no servidor e publica no tópico
        returnValue++;
        this->takeoffClients[droneNumber]->cancelAllGoals();
        this->poseClients[droneNumber]->cancelAllGoals();
        this->landingClients[droneNumber]->cancelAllGoals();
        this->removeCurrentGoalAndUpdate(droneNumber, CANCELED);
        }
      }
      //executa se número de drone inválido for informado
      else ROS_WARN("[WARNING] Tried to cancel current goal of nonexistent drone!");
    }

    return returnValue;
  }

  //essa função remove um objetivo e atualiza
  void CoreWaypoint::removeCurrentGoalAndUpdate(uint16_t droneNumber, ActionState state)
  {
    //cria iterador
    std::set<drone_middleware::DroneWaypoint>::iterator nextInLine;

    //publica no tópico o objetivo que vai ser removido com o estado informado
    this->WaypointStatePublisher.publish(
      this->createWaypointStateMsg(this->currentGoals[droneNumber], state));

    //executa se a fila do drone não estiver vazia
    if(!this->waypoints[droneNumber].empty())
    {
      //iterador aponta para o primeiro da fila
      nextInLine = this->waypoints[droneNumber].begin();
      //põe o comando do primeiro da fila para funcionar
      this->handleWaypoint(*nextInLine);
      //apaga o primeiro da fila
      this->waypoints[droneNumber].erase(nextInLine);
    }
    else
    //executa se fila do drone estava vazia
    {
      //se a fila estava vazia então apenas atualiza o estado
      //TODO: implementar essas 3 linhas de forma mais clara
      this->actionStates[TAKEOFF][droneNumber] = state;
      this->actionStates[POSE][droneNumber] = state;
      this->actionStates[LANDING][droneNumber] = state;
    }
  }

  void CoreWaypoint::handleWaypoint(drone_middleware::DroneWaypoint waypoint)
  {
    //coloca waypoint fornecido como objetivo atual
    this->currentGoals[waypoint.droneNumber] = waypoint;

    //resgata tipo do waypoint
    ActionType type = getActionType(waypoint);
    
    //invoca função apropriada de acordo com o tipo
    switch(type)
    {
      case POSE:
        this->sendWaypoint(waypoint.droneNumber, waypoint.pose);
        break;
      case TAKEOFF:
        this->takeoffDrone(waypoint.droneNumber);
        break;
      case LANDING:
        this->landDroneAt(waypoint.droneNumber, waypoint.pose, waypoint.landAt);
        break;
      case INVALID:
        //informa se algum erro interno ocorrer devido a um código inválido
        ROS_ERROR("[ERROR] This is an internal implementation error. Handling invalid waypoint.");
        break;
    }

  }

  //função que escalona nas estruturas um comando
  uint8_t CoreWaypoint::scheduleWaypoint(drone_middleware::DroneWaypoint waypoint)
  {
    //checa se droneNumber é válido
    if(waypoint.droneNumber < N_DRONES)
    {
      //resgata tipo do waypoint
      ActionType type = getActionType(waypoint);

      //checa se tipo do waypoint é válido
      if(type != INVALID)
      {
        //checa se a fila do drone está vazia
        if(this->waypoints[waypoint.droneNumber].empty())
        {
          //checa se o seqNum informado foi 0, se sim, altera o mesmo
          if(waypoint.seqNum == 0)
            waypoint.seqNum = this->currentGoals[waypoint.droneNumber].seqNum + 1;

          //ROS_DEBUG("EMPTY"); //DEBUG

          //checa se o drone pode receber comando agora
          //só pode receber se não estiver com comando ativo nem comando em falha
          if(this->isAbleToReceiveActions(waypoint.droneNumber))
          {
            //se sim, executa handleWaypoint
            this->handleWaypoint(waypoint);
          }
          else
          {
            //se não, coloca comando na fila
            this->waypoints[waypoint.droneNumber].insert(
              this->waypoints[waypoint.droneNumber].begin(), waypoint);
          }
        }
        else
        //executa se fila do drone não está vazia
        {
          //checa se o seqNum informado foi 0, se sim, altera o mesmo
          if(waypoint.seqNum == 0)
            waypoint.seqNum = std::prev(this->waypoints[waypoint.droneNumber].end())->seqNum + 1;

          //ROS_DEBUG("NOT EMPTY"); //DEBUG

          //insere na fila
          this->waypoints[waypoint.droneNumber].insert(
            std::prev(this->waypoints[waypoint.droneNumber].end()), waypoint);
        }

        //ROS_DEBUG("seqNum = %i", waypoint.seqNum); //DEBUG
      }
      else
      {
        //executa se tipo do waypoint era inválido
        ROS_WARN("[WARNING] Tried to schedule a malformed waypoint command.");
        return 0;
      }
    }
    else
    {
      //executa se número do drone informado era inválido
      ROS_WARN("[WARNING] Tried to schedule waypoint for nonexistent drone.");
      return 0;
    }

    return 1;
  }

  //decola o drone (no sentido de pertimir seus movimentos subsequentes)
  void CoreWaypoint::takeoffDrone(uint16_t droneNumber)
  {
    //cria mensagem de objetivo de action
    hector_uav_msgs::TakeoffGoal goalMsg;

    //envia mensagem através do cliente takeoff apropriado e registra callbacks
    this->takeoffClients[droneNumber]->sendGoal(goalMsg, 
      boost::bind(&CoreWaypoint::takeoffGoalCallback, this, _1, _2),
      boost::bind(&CoreWaypoint::takeoffActiveCallback, this),
      boost::bind(&CoreWaypoint::takeoffFeedbackCallback, this, _1));

    //atualiza estado
    this->actionStates[TAKEOFF][droneNumber] = ACTIVE;

    //publica atualização de estado no tópico
    this->WaypointStatePublisher.publish(
      this->createWaypointStateMsg(this->currentGoals[droneNumber], ACTIVE));
  }

  //envia comandos de pose para o drone
  void CoreWaypoint::sendWaypoint(uint16_t droneNumber, geometry_msgs::Pose pose)
  {
    //numero de sequência estático do cabeçalho
    //não é o mesmo do seqNum das filas
    static uint32_t seq = 0;

    //mensagem de objetivo de action
    hector_uav_msgs::PoseGoal goalMsg;
    std::stringstream ss;

    //limpar o stream e colocar nele a frame_id que depende do número do drone
    ss.str(std::string());
    ss << this->prefix << droneNumber << "/world";

    //construir cabeçalho e adicionar a informação pose
    goalMsg.target_pose.header.seq = seq;
    goalMsg.target_pose.header.stamp = ros::Time::now();
    goalMsg.target_pose.header.frame_id = ss.str();
    goalMsg.target_pose.pose = pose;

    //envia mensagem através do cliente takeoff apropriado e registra callbacks
    this->poseClients[droneNumber]->sendGoal(goalMsg, 
      boost::bind(&CoreWaypoint::poseGoalCallback, this, _1, _2),
      boost::bind(&CoreWaypoint::poseActiveCallback, this),
      boost::bind(&CoreWaypoint::poseFeedbackCallback, this, _1));

    //atualiza estado
    this->actionStates[POSE][droneNumber] = ACTIVE;

    //publica atualização de estado no tópico
    this->WaypointStatePublisher.publish(
      this->createWaypointStateMsg(this->currentGoals[droneNumber], ACTIVE));
  }

  void CoreWaypoint::landDroneAt(uint16_t droneNumber, geometry_msgs::Pose landingPose, uint8_t landAt)
  {
    //numero de sequência estático do cabeçalho
    //não é o mesmo do seqNum das filas
    static uint32_t seq = 0;

    //mensagem de objetivo de action
    hector_uav_msgs::LandingGoal goalMsg;
    std::stringstream ss;
    
    //limpar o stream e colocar nele a frame_id que depende do número do drone
    //se landAt for falso, frame_id sera string vazia
    //frame_id inválida faz com que as coordenadas enviadas sejam ignoradas
    //e portanto que o drone pouse diretamente abaixo
    //essa é a maneira que os desenvolvedores do hector_quadrotor implementaram
    ss.str(std::string());
    //if(landAt) ss << "drone" << droneNumber << "/world";
    ss << "drone" << droneNumber << "/world";

    //checagem do landAt e construção do cabeçalho completo ou incompleto e com ou sem pose
    if(landAt)
    {
      goalMsg.landing_zone.header.seq = seq;
      goalMsg.landing_zone.header.stamp = ros::Time::now();
      goalMsg.landing_zone.header.frame_id = ss.str();
      goalMsg.landing_zone.pose = landingPose;
    }
    else
    {
      goalMsg.landing_zone.header.frame_id = std::string();
    }

    //envia mensagem através do cliente takeoff apropriado e registra callbacks
    this->landingClients[droneNumber]->sendGoal(goalMsg, 
      boost::bind(&CoreWaypoint::landingGoalCallback, this, _1, _2),
      boost::bind(&CoreWaypoint::landingActiveCallback, this),
      boost::bind(&CoreWaypoint::landingFeedbackCallback, this, _1));

    //atualiza estado
    this->actionStates[LANDING][droneNumber] = ACTIVE;

    //publica atualização de estado no tópico
    this->WaypointStatePublisher.publish(
      this->createWaypointStateMsg(this->currentGoals[droneNumber], ACTIVE));
  }

  void CoreWaypoint::setMaximumAttempts(uint8_t newValue)
  {
    //simples setter
    this->maximumFailedAttempts = newValue;

    ROS_INFO("[MAXIMUM_ATTEMPTS] Set maximum attempts at completing waypoint command to %i.", newValue);
  }

  CoreWaypoint::ActionType CoreWaypoint::getActionType(drone_middleware::DroneWaypoint waypoint)
  {
    //operação booleana entre os flags para juntá-los
    uint8_t key = (waypoint.takeoff << 2) | (waypoint.land << 1) | (waypoint.landAt);

    //determina se é takeoff, pose, landing ou inválido
    switch(key)
    {
      case 0x04:
        return TAKEOFF;
      case 0x00:
        return POSE;
      case 0x02:
      case 0x03:
        return LANDING;
      default:
        return INVALID;
    }
  }

  void CoreWaypoint::sendTwistLimit(uint8_t all, uint16_t droneNumber, geometry_msgs::Twist limit)
  {
    //checa se ação é para todos os drones
    if(all)
    {
      for(int droneNumber = 0; droneNumber < N_DRONES; droneNumber++)
      {
        //publica no tópico do hector_quadrotor que limita velocidades do drone específico
        this->twistLimitPublishers[droneNumber].publish(limit);
      }

      ROS_INFO("[TWIST_LIMIT] Twist limit set for all drones as (%g, %g, %g, %g, %g, %g).", 
        limit.linear.x, limit.linear.y, limit.linear.z, limit.angular.x, limit.angular.y, limit.angular.z);
    }
    else
    {
      //publica no tópico do hector_quadrotor que limita velocidades do drone específico
      this->twistLimitPublishers[droneNumber].publish(limit);

      ROS_INFO("[TWIST_LIMIT] Twist limit set for drone%i as (%g, %g, %g, %g, %g, %g).", droneNumber, 
        limit.linear.x, limit.linear.y, limit.linear.z, limit.angular.x, limit.angular.y, limit.angular.z);
    }
  }

  //essa função serve para simplificar a criação dessa mensagem apenas
  drone_middleware::WaypointState CoreWaypoint::createWaypointStateMsg(
    drone_middleware::DroneWaypoint waypoint, ActionState state)
  {
    drone_middleware::WaypointState wpState;

    wpState.droneNumber = waypoint.droneNumber;
    wpState.seqNum = waypoint.seqNum;
    wpState.pose = waypoint.pose;
    wpState.takeoff = waypoint.takeoff;
    wpState.land = waypoint.land;
    wpState.landAt = waypoint.landAt;
    wpState.actionState = state;

    return wpState;
  }

  //Determina se o drone pode receber uma nova ação agora
  //pode receber se sua última ação está como concluída com sucesso ou como cancelada
  //se estiver como ativa ou falha, não é possível
  uint8_t CoreWaypoint::isAbleToReceiveActions(uint16_t droneNumber)
  {
    //resgata objetivo atual com nome de waypoint
    drone_middleware::DroneWaypoint waypoint = this->currentGoals[droneNumber];
    //resgata estado do tipo de ação do objetivo atual para esse drone
    uint8_t state = this->actionStates[this->getActionType(waypoint)][droneNumber];

    //ROS_DEBUG("[DBG] seqnum = %i; state = %i", waypoint.seqNum, state); //DEBUG

    //seqnum == 0 indica que é o objetivo inicial placeholder, que pode ser substituído
    if(waypoint.seqNum == 0) return 1;
    //SUCCEEDED e CANCELED indicam estados que podem ser substituídos
    else if(state == (uint8_t) SUCCEEDED || state == (uint8_t) CANCELED) return 1;
    //caso contrário, falso
    else return 0;
  }

  //#endregion

}