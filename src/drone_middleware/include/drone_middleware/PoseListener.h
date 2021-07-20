#ifndef CLASS_POSELISTENER_H
#define CLASS_POSELISTENER_H

//#region [include]

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"

#include "drone_middleware/DronePose.h"
#include "drone_middleware/PoseRequest.h"
#include "drone_middleware/MobilityHeatMap.h"
#include "drone_middleware/GetHeatMapData.h"

#include <stdio.h>
#include <sstream>
#include <vector>

//#endregion

//#region [class]

namespace drone_middleware
{

  //! Classe que monitora posição dos drones.
  /*!
  *   *	Essa classe monitora posição dos drones e também fornece o serviço de informar essa posição.
  */
  class PoseListener
  {

    private:

    //! Número de drones na simulação.
    /*!
    *   *	Número de drones, passado como parâmetro ROS por script ou linha de comando, usar a função estática setAndGetNDrones para atribuir valor.
    */
    static int N_DRONES;

    //! Vetor de estado atual dos drones.
    /*!
    *   *	Guarda a posição em que cada drone atualmente está.
    */
    std::vector<drone_middleware::DronePose> droneArray;

    //! ROS Node Handle.
    /*!
    *   *	"Alça" pela qual o objeto se conecta ao ROS.
    *   *   NodeHandle deve ser criada apenas após o ros::init.
    */
    ros::NodeHandle n;

    //! Servidor de posição de drone.
    /*!
    *   *	Consulta a estrutura de dados droneArray para fornecer a posição atual de qualquer drone.
    */
    ros::ServiceServer poseRequestService;

    //! Servidor de fornecimento do heatmap.
    /*!
    *   *	Fornece os dados do heatmap.
    */
    ros::ServiceServer getHeatMapDataService;

    //! Vetor de subscribers dos drones.
    /*!
    *   *	Subscribers que estão a todo tempo lendo as atualizações de posição publicadas por todos os drones.
    */
    std::vector<ros::Subscriber> droneSubscribers;

    //! Classe que armazena os dados do heatmap.
    /*!
    *   *	Ver o arquivo da classe para maiores detalhes: "drone_middleware/MobilityHeatMap.h".
    */
    drone_middleware::MobilityHeatMap * mobilityHeatMap;

    public:

    //! Construtor da classe.
    /*!
    *   *	Possui muitos argumentos para permitir flexibilidade. Na falta de necessidade para tal basta usar os valores padrão.
    *   *   nodeName: nome do nó e prefixo dos tópicos/serviços etc associados a ele. (padrão: pose_listener)
    *   *   poseServiceName: nome do serviço oferecido. (padrão: pose_request)
    *   *   topicPrefix: prefixo que é utilizado nos namespaces dos drones. (padrão: drone)
    *   *   topicSuffix: tópico que recebe as mensagens geometry_msgs::PoseStamped. (padrão: /ground_truth_to_tf/pose)
    *   *   bufferSize: tamanho da fila de mensagens a ser usada nas comunicações ROS. (padrão: 10) (Valor padrão escolhido sem um critério definido, outro valor pode ser melhor ou não)
    */
    PoseListener(std::string nodeName, std::string poseServiceName, std::string topicPrefix, 
      std::string topicSuffix, unsigned int bufferSize, std::string heatMapServiceName, bool heatMapActive, 
      double heatMapXMin, double heatMapYMin, double heatMapWidth, uint16_t heatMapNCells);

    //! Configuração de número de drones
    /*!
    *   *	Chamar no para configurar número de drones a partir do servidor de parâmetros do ROS.
    *   *   Parâmetro padrão: /drone_middleware/n_drones
    */
    static int setAndGetNDrones(std::string paramName);

    //! Limpar e inicializar droneArray
    /*!
    *   *	Limpa e inicializa a estrutura de dados droneArray.
    */
    void resetDroneArray();

    //! Callback de receber posição.
    /*!
    *   *	Callback que trata recebimento de mensagens PoseStamped publicadas pelos drones.
    *   *   Callbacks não são chamados diretamente.
    */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    //! Callback de serviço de posição.
    /*!
    *   *	Callback que trata de fornecer posição dos drones.
    *   *   Callbacks não são chamados diretamente.
    */
    bool poseRequestCallback(drone_middleware::PoseRequest::Request  &req,
      drone_middleware::PoseRequest::Response &res);

    //! Callback de serviço de heatmap.
    /*!
    *   *	Callback que trata de fornecer dados do heatmap.
    *   *   Callbacks não são chamados diretamente.
    */
    bool getHeatMapDataCallback(drone_middleware::GetHeatMapData::Request  &req,
      drone_middleware::GetHeatMapData::Response &res);

  };

//#endregion

}

#endif