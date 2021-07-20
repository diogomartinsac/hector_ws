#ifndef CLASS_CORETELEOPERATOR_H
#define CLASS_CORETELEOPERATOR_H

//#region [include]

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "rosgraph_msgs/Clock.h"

#include "hector_uav_msgs/EnableMotors.h"

#include "drone_middleware/DroneMovement.h"
#include "drone_middleware/ScheduleMovement.h"

#include <sstream>
#include <string>
#include <iostream>
#include <fstream>
#include <queue>

//#endregion

//#region [class]

namespace drone_middleware
{
  //! Classe que recebe solicitações de comandos de movimento para os drones, os escalona e envia no momento correto para os drones.
  /*!
  *   *	A classe mantem uma fila de prioridade com os movimentos ordenados por tempo de início e o estado de movimento atual dos drones.
  *   *   Essa classe é usada por todos os outros Teleoperators como servidor do serviço de escalonar movimentos.
  */
  class CoreTeleoperator
  {

    private:

    //! Número de drones na simulação.
		/*!
		*   *	Número de drones, passado como parâmetro ROS por script ou linha de comando, usar a função estática setAndGetNDrones para atribuir valor.
		*/
    static int N_DRONES;

    //! Vetor de movimentos atualmente sendo enviados aos drones.
		/*!
		*   *	O comando é continuamente enviado para o drone na taxa do tópico /cmd_vel.
		*/
    std::vector<drone_middleware::DroneMovement> droneStates;

    //! Fila de movimentos ordenados.
		/*!
		*   *	É uma fila de prioridade com os comandos de movimentos, ordenada pela friend function presente na classe DroneMovement.
		*/
    std::priority_queue<drone_middleware::DroneMovement, 
      std::vector<drone_middleware::DroneMovement>, 
      std::greater<drone_middleware::DroneMovement>> movements;

    //! Booleano que indica se está publicando.
		/*!
		*   *	Somente envia comandos aos drones se esse booleano estiver habilitado.
    *   *   Se estiver desabilitado, os comandos são descartados ao chegar no tempo.
		*/
    bool publishing;

    //! ROS Node Handle.
		/*!
		*   *	"Alça" pela qual o objeto se conecta ao ROS.
    *   *   NodeHandle deve ser criada apenas após o ros::init.
		*/
    ros::NodeHandle n;

    //! Publicadores de comandos de movimentos.
		/*!
		*   *	Um vetor de publicadores, um para cada drone, publicam as mensagens no tópico /cmd_vel.
		*/
    std::vector<ros::Publisher> dronePublishers;
    
    //! Servidor que escalona os movimentos.
		/*!
		*   *	Fornece o serviço de escalonar os movimentos, atendendo às chamadas de outros Teleoperators.
		*/
    ros::ServiceServer scheduleMovementService;
    
    //! Clientes do serviço de iniciar motores dos drones.
		/*!
		*   *	Antes de movimentar é necessário partir os motores chamando um serviço em cada drone com esses clientes.
		*/
    std::vector<ros::ServiceClient> motorStarterClients;

    //! Subscriber do tópico /clock.
		/*!
		*   *	Esse subscriber recebe as atualizações de relógio publicadas no tópico /clock (que se originam do Gazebo) e se utiliza delas para comandar os drones no instante correto.
		*/ 
    ros::Subscriber clockSubscriber;

    public:

    //! Construtor da classe.
		/*!
		*   *	Possui muitos argumentos para permitir flexibilidade. Na falta de necessidade para tal basta usar os valores padrão.
    *   *   nodeName: nome do nó e prefixo dos tópicos/serviços etc associados a ele. (padrão: drone_teleop_core)
    *   *   serviceName: nome do serviço oferecido. (padrão: schedule_movement)
    *   *   clockTopic: tópico que fornece a cadência do tempo. (padrão: /clock)
    *   *   prefix: prefixo que é utilizado nos namespaces dos drones. (padrão: drone)
    *   *   topicSuffix: tópico que recebe as mensagens geometry_msgs::Twist. (padrão: cmd_vel)
    *   *   serviceSuffix: serviço que liga os motores dos drones. (padrão: enable_motors)
    *   *   bufferSize: tamanho da fila de mensagens a ser usada nas comunicações ROS. (padrão: 10) (Valor padrão escolhido sem um critério definido, outro valor pode ser melhor ou não)
		*/
    CoreTeleoperator(std::string nodeName, std::string serviceName, 
      std::string clockTopic, std::string prefix, std::string topicSuffix, 
      std::string serviceSuffix, unsigned int bufferSize);

    //! Configuração de número de drones
		/*!
		*   *	Chamar no para configurar número de drones a partir do servidor de parâmetros do ROS.
    *   *   Parâmetro padrão: /drone_middleware/n_drones
		*/ 
    static int setAndGetNDrones(std::string paramName);

    //! Limpar e inicializar droneStates
		/*!
		*   *	Limpa e inicializa a estrutura de dados droneStates.
		*/
    void resetDroneStates();

    //! Callback de escalonar movimento.
		/*!
		*   *	Insere movimento na fila.
    *   *   Callbacks não são chamados diretamente.
		*/
    bool scheduleMovementCallback(drone_middleware::ScheduleMovement::Request  &req, 
      drone_middleware::ScheduleMovement::Response &res);

    //! Callback do relógio.
		/*!
		*   *	É chamado periodicamente com informações de tempo.
    *   *   Callbacks não são chamados diretamente.
		*/
    void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg);

    //! Iniciar motores dos drones.
		/*!
		*   *	Deve ser chamado para iniciar os motores antes de começar a operar os drones.
		*/
    void startDroneMotors();

    //! Começar a publicar para os drones.
		/*!
		*   *	É um setter de booleano basicamente.
		*/
    void startPublishing();

    //! Parar de publicar para os drones.
		/*!
		*   *	É um setter de booleano basicamente.
		*/
    void stopPublishing();

  };

//#endregion

}

#endif