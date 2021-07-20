//#region [include]

#include "drone_middleware/PoseListener.h"

//#endregion

namespace drone_middleware
{

  //#region [class]

  //numero de drones, deve ser alterado caso queira monitorar mais drones
  int PoseListener::N_DRONES;

  //vetor de classes de posicao de drone, para ser acessado no callback
  //std::vector<drone_middleware::DronePose> PoseListener::droneArray;

  //construtor
  PoseListener::PoseListener(std::string nodeName, std::string poseServiceName, std::string topicPrefix, 
    std::string topicSuffix, unsigned int bufferSize, std::string heatMapServiceName, bool heatMapActive, 
    double heatMapXMin, double heatMapYMin, double heatMapWidth, uint16_t heatMapNCells)
  {

    //criando o heatmap
    this->mobilityHeatMap = new drone_middleware::MobilityHeatMap(heatMapActive, heatMapXMin, heatMapYMin, 
      heatMapWidth, heatMapNCells, N_DRONES);

    this->resetDroneArray();

    this->poseRequestService = this->n.advertiseService(
      nodeName + "/" + poseServiceName, &PoseListener::poseRequestCallback, this);

    this->getHeatMapDataService = this->n.advertiseService(
      nodeName + "/" + heatMapServiceName, &PoseListener::getHeatMapDataCallback, this);

    //loop para realizar acoes para todos os drones
    std::stringstream ss;
    for(int i = 0; i < N_DRONES; i++)
    {

      //this->droneArray.push_back(drone_middleware::DronePose());
      
      //inicializando id sequencial armazenado para cada drone
      //this->droneArray[i].dronePose.header.seq = 0;
      
      //limpar o stream e colocar nele o número atual do loop
      ss.str(std::string());
      ss << i;
      //cada leitor se inscreve no topico de um drone
      this->droneSubscribers.push_back(this->n.subscribe(topicPrefix + ss.str() + 
        topicSuffix, bufferSize, &PoseListener::poseCallback, this));
    }
    
  }

  //#endregion

  //#region [func]

  //a partir do servidor de parametros ros
  int PoseListener::setAndGetNDrones(std::string paramName)
  {
    //pegando parametro do numero de drones
    if(!ros::param::get(paramName, N_DRONES))
    {
      ROS_ERROR("[PARAM] Unable to get the ROS parameter %s", paramName.c_str());
      return -1;
    }
    else return N_DRONES;
  }

  void PoseListener::resetDroneArray()
    {
      this->droneArray.clear();

      //loop para realizar acoes para todos os drones
      for(int i = 0; i < N_DRONES; i++)
      {
        this->droneArray.push_back(drone_middleware::DronePose());
        
        //inicializando id sequencial armazenado para cada drone
        this->droneArray[i].dronePose.header.seq = 0;
      }
    }

  //#endregion

  //#region [callback]

  //callback que trata recebimento de mensagens PoseStamped publicadas pelos drones
  void PoseListener::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    int droneNumber;
    drone_middleware::HeatMapCell currentCell, lastCell;

    //descobre numero do drone que mandou a mensagem
    sscanf(msg->header.frame_id.c_str(), "drone%i/world", &droneNumber);
    
    //retorna se o identificador sequencial da mensagem recebida é menor que o armazenado
    if(msg->header.seq < this->droneArray[droneNumber].dronePose.header.seq) return;
    
    this->droneArray[droneNumber].dronePose = *msg;
    this->droneArray[droneNumber].droneNumber = droneNumber;

    //se heatmap ativo
    if(this->mobilityHeatMap->is_active())
    {
      //checagem da célula para heatmap
      currentCell = this->mobilityHeatMap->getCellForPoint(
        this->droneArray[droneNumber].dronePose.pose.position.x, 
        this->droneArray[droneNumber].dronePose.pose.position.y);

      lastCell = this->mobilityHeatMap->getDroneLastCell(droneNumber);

      //se alterou entra aqui
      if(currentCell.i != lastCell.i || currentCell.j != lastCell.j)
      {
        //altera last cell e incrementa current cell
        this->mobilityHeatMap->setDroneLastCell(droneNumber, currentCell);
        this->mobilityHeatMap->incrementCell(currentCell);
      }
    }

    //imprime como informativo
    #ifdef DISPLAY_ROS_INFO
    ROS_INFO("[POSE] [drone: %i] [seq: %u] [%gx, %gy, %gz]\n", 
      droneNumber, 
      this->droneArray[droneNumber].dronePose.header.seq, 
      this->droneArray[droneNumber].dronePose.pose.position.x, 
      this->droneArray[droneNumber].dronePose.pose.position.y, 
      this->droneArray[droneNumber].dronePose.pose.position.z);
    #endif

  }

  //callback que trata de fornecer posicao dos drones
  bool PoseListener::poseRequestCallback(drone_middleware::PoseRequest::Request  &req,
    drone_middleware::PoseRequest::Response &res)
  {

    if(req.all)
    {
      for(int droneNumber = 0; droneNumber < N_DRONES; droneNumber++)
      {
        res.currentPose.push_back(this->droneArray[droneNumber].dronePose);
      }
    }
    else if(req.droneNumber < this->droneArray.size())
    {
      res.currentPose.push_back(this->droneArray[req.droneNumber].dronePose);
    }
    else
    {
      ROS_WARN("Requested pose of nonexistent drone%i.", req.droneNumber);
    }
    
    return true;

  }

  bool PoseListener::getHeatMapDataCallback(drone_middleware::GetHeatMapData::Request  &req,
    drone_middleware::GetHeatMapData::Response &res)
  {
    res.x_min = this->mobilityHeatMap->get_x_min();
    res.y_min = this->mobilityHeatMap->get_y_min();
    res.width = this->mobilityHeatMap->get_width();
    res.n_cells = this->mobilityHeatMap->get_n_cells();

    res.heatMap = this->mobilityHeatMap->getHeatMapAsVector();

    //ROS_INFO("[DEBUG] %f %f %f %i", res.x_min, res.y_min, res.width, res.n_cells);

    return true;
  }  

  //#endregion

}