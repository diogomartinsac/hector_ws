#ifndef CLASS_WORLDCONTROLLER_H
#define CLASS_WORLDCONTROLLER_H

//#region [include]

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/server/action_server.h"

#include "drone_middleware/Pause.h"
#include "drone_middleware/Unpause.h"
#include "drone_middleware/Restart.h"
#include "drone_middleware/MultiStepAction.h"
#include "drone_middleware/GetSimInfo.h"

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>
#include <string>
#include <cstdint>

//#endregion

//#region [class]

namespace drone_middleware
{
  //! Estrutura que guarda informações sobre a simulação atual.
  /*!
  *   *	ros::WallTime é tempo real enquanto que ros::Time é tempo virtual, simulado.
  */
  typedef struct gazeboSimInfo
  {

  ros::WallTime startTime;
  ros::WallTime realTime;
  ros::Time simTime;
  ros::Time pauseTime;
  std::uint64_t iterations;
  std::uint64_t iterationGoal;
  bool paused;
  bool stepping;

  } gazeboSimInfo;

  //! Estrutura que guarda informações sobre o passo de simulação atual.
  /*!
  *   *	Similar a gazeboSimInfo.
  */
  typedef struct stepInfo
  {

  uint64_t stepIterations;
  ros::Time stepTime;
  ros::WallTime stepTimeReal;

  } stepInfo;

  //! Classe que controla a física e o tempo da simulação do gazebo.
  /*!
  *   *	Possibilita parar, iniciar, reiniciar e prosseguir uma quantidade finita determinada de tempo na simulação, além de permitir obter dados sobre a mesma.
  *   *   O Gazebo tem sua própria estrutura de tópicos independente da do ROS, essa classe está na interseção desses dois domínios.
  */
  class WorldController
  {

    private:

    //! Estrutura que guarda informações sobre a simulação atual.
    /*!
    *   *	Ver gazeboSimInfo.
    */
    gazeboSimInfo currentSim;
    
    //! Estrutura que guarda informações sobre o passo de simulação atual.
    /*!
    *   *	Ver stepInfo.
    */
    stepInfo currentStep;

    //! Publisher que envia comandos de tempo para o gazebo.
    /*!
    *   *	Publisher que envia os comandos de tempo para o gazebo no tópico ~/world_control.
    *   *   Ver mais em: <https://osrf-distributions.s3.amazonaws.com/gazebo/msg-api/dev/world__control_8proto.html>
    */
    gazebo::transport::PublisherPtr controlPublisher;

    //! Publisher que envia comandos de física para o gazebo.
    /*!
    *   *	Publisher que envia os comandos de física para o gazebo no tópico ~/physics.
    *   *   Ver mais em: <https://osrf-distributions.s3.amazonaws.com/gazebo/msg-api/dev/physics_8proto.html>
    */
    gazebo::transport::PublisherPtr physicsPublisher;

    //! Ponteiro de nó do Gazebo.
    /*!
    *   *	Tem função similar à ROS Node Handle.
    */
    gazebo::transport::NodePtr node;

    //! Subscriber que recebe status da simulação do gazebo.
    /*!
    *   *	Subscriber que recebe status da simulação do gazebo no tópico ~/world_stats.
    *   *   Ver mais em: <https://osrf-distributions.s3.amazonaws.com/gazebo/msg-api/dev/world__stats_8proto.html>
    */
    gazebo::transport::SubscriberPtr statSubscriber;

    //! ROS Node Handle.
    /*!
    *   *	"Alça" pela qual o objeto se conecta ao ROS.
    *   *   NodeHandle deve ser criada apenas após o ros::init.
    */
    ros::NodeHandle n;

    //! Servidor de pause.
    /*!
    *   *	Para a simulação do Gazebo até segunda ordem.
    */
    ros::ServiceServer pauseService;

    //! Servidor de unpause.
    /*!
    *   *	Retoma a simulação do Gazebo até segunda ordem.
    */
    ros::ServiceServer unpauseService;

    //! Servidor de restart.
    /*!
    *   *	Reinicia o tempo da simulação e pausa a mesma até segunda ordem.
    */
    ros::ServiceServer restartService;

    //! Servidor de informações da simulação.
    /*!
    *   *	Fornece a estrutura currentSim do tipo gazeboSimInfo via serviço
    */
    ros::ServiceServer getSimInfoService;

    //! Servidor de ações.
    /*!
    *   *	Implementa o serviço de dar passos na simulação do gazebo de maneira não bloqueante e assíncrona usando actions.
    *   *   Não usa callbacks.
    *   *   Ver mais em: <https://docs.ros.org/kinetic/api/actionlib/html/classactionlib_1_1SimpleActionServer.html>
    *   *   Ver mais em: <http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29>
    */
    actionlib::SimpleActionServer<drone_middleware::MultiStepAction> * multiStepServer;

    public:

    //! Construtor da classe.
    /*!
    *   *	  Possui muitos argumentos para permitir flexibilidade. Na falta de necessidade para tal basta usar os valores padrão.
    *   *   nodeName: nome do nó e prefixo dos tópicos/serviços etc associados a ele. (padrão: gazebo_world_controller)
    *   *   pauseName: nome do serviço de pause. (padrão: pause)
    *   *   unpauseName: nome do serviço de unpause. (padrão: unpause) 
    *   *   restartName: nome do serviço de restart. (padrão: restart)
    *   *   getSimInfoName: nome do serviço de informações. (padrão: get_sim_info)
    *   *   multiStepAction: nome da ação de step. (padrão: multi_step)
    *   *   worldStatsTopic: nome do tópico de status da simulação. (padrão: ~/world_stats)
    *   *   worldControlTopic: nome do tópico de controle da simulação. (padrão: ~/world_control)
    *   *   physicsTopic: nome do tópico de física da simulação. (padrão: ~/physics)
    *   *   gazeboBufferSize: tamanho da fila de mensagens a ser usada nas comunicações Gazebo. (padrão: 10) (Valor padrão escolhido sem um critério definido, outro valor pode ser melhor ou não)
    *   *   gazeboTopicRate: frequência de envio mensagens a ser usada nas comunicações Gazebo. (padrão: 1000) (Valor padrão escolhido sem um critério definido, outro valor pode ser melhor ou não)
    */
    WorldController(std::string nodeName, std::string pauseName, 
    std::string unpauseName, std::string restartName, std::string getSimInfoName,
    std::string multiStepAction, std::string worldStatsTopic, 
    std::string worldControlTopic, std::string physicsTopic, 
    unsigned int gazeboBufferSize, unsigned int gazeboTopicRate);

    //! Aguarda conexão nos publicadores.
    /*!
    *   *	Faz com que os publicadores aguardem conexão com o Gazebo.
    */
    void waitConnectionOnPublishers();

    //! Pause.
    /*!
    *   *	Efetua o pause e bloqueia até confirmar esta ação.
    */
    void pause();

    //! Unpause.
    /*!
    *   *	Efetua o unpause e bloqueia até confirmar esta ação.
    */
    void unpause();

    //! Restart.
    /*!
    *   *	Efetua o restart e bloqueia até confirmar esta ação.
    */
    void restart();

    //! MultiStep.
    /*!
    *   *	Passa o comando de dar passos na simulação mas não bloqueia, resultado é enviado depois dentro do callback de ~/world_stats quando for constatado que o objetivo for atingido.
    */
    void multiStep(uint32_t steps);

    //! Real time update rate.
    /*!
    *   *	O quanto o Gazebo espera por cada passo, pode ser utilizado para rodar a simulação em tempo real ou mais rápido se possível.
    *   *   Se real_time_update_rate = 0, Gazebo simula tão rápido quanto possível
    */
    void setRealTimeUpdateRate(float rate);

    //! Callback de status da simulação.
    /*!
    *   *	Obtém informações sobre a simulação do Gazebo.
    *   *   Callbacks não são chamados diretamente.
    */
    void worldStatsCallback(ConstWorldStatisticsPtr &_msg);

    //! Callback de pause.
    /*!
    *   *	Faz uso da função pause().
    *   *   Callbacks não são chamados diretamente.
    */
    bool pauseCallback(drone_middleware::Pause::Request  &req,
      drone_middleware::Pause::Response &res);

    //! Callback de unpause.
    /*!
    *   *	Faz uso da função unpause().
    *   *   Callbacks não são chamados diretamente.
    */
    bool unpauseCallback(drone_middleware::Unpause::Request  &req,
      drone_middleware::Unpause::Response &res);

    //! Callback de restart.
    /*!
    *   *	Faz uso da função restart().
    *   *   Callbacks não são chamados diretamente.
    */
    bool restartCallback(drone_middleware::Pause::Request  &req,
      drone_middleware::Pause::Response &res);

    //! Callback de multiStep.
    /*!
    *   *	Faz uso da função multiStep().
    *   *   Callbacks não são chamados diretamente.
    */
    void multiStepCallback(
      const actionlib::SimpleActionServer<drone_middleware::MultiStepAction>::GoalConstPtr& goal);

    //! Callback de fornecer informações de simulação.
    /*!
    *   *	  Fornece a estrutura que armazena o estado da simulação.
    */
    bool getSimInfoCallback(drone_middleware::GetSimInfo::Request  &req,
      drone_middleware::GetSimInfo::Response &res);
      
  };

//#endregion

}

#endif