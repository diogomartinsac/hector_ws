#ifndef CLASS_COREWAYPOINT_H
#define CLASS_COREWAYPOINT_H

//#region [include]

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//#include "rosgraph_msgs/Clock.h"
#include "actionlib/client/simple_action_client.h"

//#include "hector_uav_msgs/EnableMotors.h"
#include "hector_uav_msgs/TakeoffAction.h"
#include "hector_uav_msgs/PoseAction.h"
#include "hector_uav_msgs/LandingAction.h"

#include "drone_middleware/DroneWaypoint.h"
//#include "drone_middleware/ScheduleMovement.h"
#include "drone_middleware/WaypointState.h"
#include "drone_middleware/ScheduleWaypoint.h"
#include "drone_middleware/CancelWaypoints.h"
#include "drone_middleware/GetCurrentGoal.h"
#include "drone_middleware/SetMaximumAttempts.h"
#include "drone_middleware/SetTwistLimit.h"

#include <sstream>
#include <string>
//#include <iostream>
//#include <fstream>
//#include <queue>
#include <cstdint>

//#endregion

//#region [class]

namespace drone_middleware
{
  //! Classe que recebe solicitações de comandos de posição para os drones, os escalona e envia no momento correto para os drones.
  /*!
  *   *	A classe mantem uma estrutura ordenada para cada drone com as posições em ordem de envio.
  *   *   Essa classe é usada por todos os outros controladores Waypoint como servidor do serviço de escalonar comandos de posição.
  */
  class CoreWaypoint
  {

    private:

    //! Indica tipos de clientes de ação que estão sendo utilizados (ver descrição).
		/*!
		*   *	Ver a descrição da variável "actionStates", única função desta enum é dar suporte a este outro membro da classe.
		*/
    enum ActionType {
      TAKEOFF = 0,
      POSE = 1,
      LANDING = 2,
      INVALID = 3
    };

    //! Indica estado do objetivo atual de um drone.
		/*!
    *   *   CANCELLED = -2 (ação foi explicitamente cancelada via requisição e não há mais ações na fila) (o controlador está ocioso para esse drone)
    *   *   FAILED = -1 (falhou além do limite e parou de tentar novamente) (o controlador está ocioso para esse drone).
    *   *   ACTIVE = 0 (ação está em curso e ainda não falhou nenhuma vez) (o controlador está ativo para esse drone).
    *   *   SUCCEEDED = 1 (ação foi completada e não há mais ações em espera na fila) (o controlador está ocioso para esse drone).
		*/
    enum ActionState {
      CANCELED = -2,
      FAILED = -1,
      ACTIVE = 0,
      SUCCEEDED = 1
    };

    //! Número de drones na simulação.
		/*!
		*   *	Número de drones, passado como parâmetro ROS por script ou linha de comando, usar a função estática setAndGetNDrones para atribuir valor.
		*/
    static int N_DRONES;

    //! Fila de posições ordenadas.
		/*!
		*   *	É uma estrutura com os comandos de posição, ordenada pela friend function presente na classe DroneWaypoint.
		*/
    std::vector< std::set< drone_middleware::DroneWaypoint,  
      std::less< drone_middleware::DroneWaypoint > > > waypoints;

    //! Atuais objetivos dos drones.
		/*!
		*   *	É um vetor que contem os objetivos de cada drone.
		*/
    std::vector<drone_middleware::DroneWaypoint> currentGoals;

    //! ROS Node Handle.
		/*!
		*   *	"Alça" pela qual o objeto se conecta ao ROS.
    *   *   NodeHandle deve ser criada apenas após o ros::init.
		*/
    ros::NodeHandle n;
    
    //! Cliente de ações TakeoffAction.
		/*!
		*   *	Implementa a função de takeoff do drone.
    *   *   Ver mais em: <https://docs.ros.org/kinetic/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html>
    *   *   Ver mais em: <http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient>
		*/
    std::vector< actionlib::SimpleActionClient<hector_uav_msgs::TakeoffAction> * > takeoffClients;

    //! Cliente de ações PoseAction.
		/*!
		*   *	Implementa a função de controlar posição do drone.
    *   *   Ver mais em: <https://docs.ros.org/kinetic/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html>
    *   *   Ver mais em: <http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient>
		*/
    std::vector< actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> * > poseClients;

    //! Cliente de ações LandingAction.
		/*!
		*   *	Implementa a função de pousar o drone.
    *   *   Ver mais em: <https://docs.ros.org/kinetic/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html>
    *   *   Ver mais em: <http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient>
		*/
    std::vector< actionlib::SimpleActionClient<hector_uav_msgs::LandingAction> * > landingClients;

    //! Indica quais clientes de ação estão sendo utilizados (ver descrição).
		/*!
		*   *	Esta variavél foi introduzida por questões de implementação, ela é uma maneira de evitar uma mensagem de erro desnecessária.
    *   *   Esse problema já foi resolvido em implementações mais recentes da actionlib, conforme pode ser visto nos links abaixo.
    *   *   Adicionalmente acabou sendo utilizada para fornecer mais informações sobre o estado de uma ação.
    *   *   Ver mais em: <https://github.com/ros/actionlib/issues/40>
    *   *   Ver mais em: <https://github.com/ros/actionlib/pull/97>
		*/
    std::vector<uint8_t> actionStates[3];

    //! Indica quantas vezes a ação atual já falhou para um dado drone
		/*!
		*   *	Guarda um valor inteiro sem sinal para cada drone.
		*/
    std::vector<uint8_t> failCounters;

    //! Essa variável determina o número máximo de falhas antes de parar para qualquer comando
		/*!
		*   *	Se o comando falhar "maximumFailedAttempts" vezes, ele não será reenviado e a fila desse drone em específico ficará paralisada.
    *   *   A fila só voltará a andar se for requisitado o cancelamento do objetivo atual.
    *   *   O menor valor para esse parâmetro é 1, já que parar após falhar zero vezes não faria sentido. 0 e 1 terão o mesmo comportamento.
		*/
    uint8_t maximumFailedAttempts;

    //! Guarda string utilizada como prefixo em alguns nomes
		/*!
		*   *	Por padrão a string é "drone", mas essa variável existe para flexibilizar e facilitar alteração futura.
		*/
    std::string prefix;

    //! Publicadores que enviam limites de velocidade para os drones
		/*!
		*   *	Envia um geometry_msgs/Twist, que informa os limites lineares e angulares.
    * 	*	Para ver mais sobre geometry_msgs/Twist consulte: \<http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Twist.html\>
		*/
    std::vector<ros::Publisher> twistLimitPublishers;

    //! Publicador que escreve no tópico todos os comandos waypoint que são completados
		/*!
		*   *	Ele escreve no tópico os dados presentes em drone_middleware::DroneWaypoint e também um estado do tipo drone_middleware::ActionState.
		*/
    ros::Publisher WaypointStatePublisher;

    //! Servidor que adiciona comandos nas estruturas correspondentes a cada drone
		/*!
		*   *	Recebe os dados equivalentes a um drone_middleware::DroneWaypoint.
    *   *   Responde um booleano 1 ou 0 dentro de um uint8 informando se adicionou corretamente.
		*/
    ros::ServiceServer scheduleWaypointServer;

    //! Servidor que cancela comandos nas estruturas correspondentes a cada drone
		/*!
    *   *   Pode cancelar apenas o objetivo atual, apenas os que estão na fila ou os dois.
    *   *   Pode agir em todos os drones ao mesmo tempo ou apenas um.
		*   *	uint8 all --> se verdadeiro, ação vale para todos os drones.
    *   *   uint16 droneNumber --> informa qual drone, porém se all estiver ativado, é ignorado.
    *   *   uint8 cancelCurrent --> cancela apenas objetivo atual.
    *   *   uint8 clearQueue --> cancela apenas os objetivos que estão na fila.
    *   *   Responde um uint8 informando quantos objetivos foram cancelados no total.
		*/
    ros::ServiceServer cancelWaypointsServer;
    
     //! Servidor que fornece objetivo atual de um drone informado e seu estado
		/*!
		*   *	Recebe o número do drone desejado.
    *   *   Responde uma mensagem drone_middleware::WaypointState, que é equivalente a drone_middleware::DroneWaypoint + drone_midlleware::ActionState.
		*/
    ros::ServiceServer getCurrentGoalServer;
    
    //! Servidor que altera o valor de máximo de falhas que um comando pode ter
		/*!
		*   *	Após esse numero, as estruturas do drone ficarão paralisadas e essa ação ficará marcada como FAILED, necessitando ser cancelada.
    *   *   Recebe como uint8 o valor desejado. 
    *   *   Não há resposta.
		*/
    ros::ServiceServer setMaximumAttemptsServer;
    
    //! Servidor que altera o limite de velocidade de movimento para um ou todos os drones
		/*!
		*   *	uint8 all --> realiza alteração para todos os drones.
    *   *   uint16 droneNumber --> número do drone, ignorado caso all seja verdadeiro.
    *   *   geometry_msgs/Twist limit --> limite na forma de velocidades lineares e angulares contidas em um geometry_msgs/Twist.
    *   *   Não há resposta.
		*/
    ros::ServiceServer setTwistLimitServer;

    //! Determina se um drone está apto ou não para receber comandos
		/*!
		*   *	Um drone NÃO ESTÁ apto se já tiver uma ação ativa, ou se sua última ação falhou maximumFailedAttempts vezes.
    *   *   Um drone ESTÁ apto se sua última ação foi completada com sucesso ou cancelada.
		*/
    uint8_t isAbleToReceiveActions(uint16_t droneNumber);

    //! Finaliza o objetivo atual e traz um novo da fila caso necessário
		/*!
		*   *	Essa função foi criada principalmente devido a um trecho de código que se repetiu algumas vezes e então se tornou uma função.
		*/
    void removeCurrentGoalAndUpdate(uint16_t droneNumber, ActionState state);

    //! Faz o tratamento de cada comando
		/*!
		*   *	Direciona o comando para a função correta de acordo com seus valores e atualiza o objetivo atual.
		*/
    void handleWaypoint(drone_middleware::DroneWaypoint waypoint);

    //! Dá a partida em um drone
		/*!
		*   *	Envia a ação TakeoffAction para um drone.
		*/
    void takeoffDrone(uint16_t droneNumber);

    //! Envia comando de posição para um drone
		/*!
		*   *	Envia a ação PoseAction para um drone.
		*/
    void sendWaypoint(uint16_t droneNumber, geometry_msgs::Pose pose);


    //! Pousa um drone em uma posição específica
		/*!
		*   *	Envia a ação LandingAction para um drone especificando onde pousar ou pousando diretamente abaixo.
    *   *   Se landAt for verdadeiro considera posição, se não, pousa diretamente abaixo.
		*/
    void landDroneAt(uint16_t droneNumber, geometry_msgs::Pose landingPose, uint8_t landAt);

    //! Retorna o tipo de um waypoint.
		/*!
		*   *	Retorna TAKEOFF, POSE ou LANDING.
		*/
    ActionType getActionType(drone_middleware::DroneWaypoint waypoint);
    
    //! Prepara uma mensagem do tipo drone_middleware::WaypointState para ser enviada
		/*!
		*   *	Retorna drone_middleware::WaypointState pronta para enviar.
		*/
    WaypointState createWaypointStateMsg(drone_middleware::DroneWaypoint waypoint, ActionState state);

    //! Callback de resultado da ação takeoff
		/*!
		*   *	Recebe e trata o resultado da ação takeoff.
		*/ 
    void takeoffGoalCallback(const actionlib::SimpleClientGoalState& state,
      const hector_uav_msgs::TakeoffResult::ConstPtr& result);

    //! Callback de atividade da ação takeoff
		/*!
		*   *	É executado quando um objetivo da ação takeoff se torna ativo.
    *   *   Na implementação atual é uma função vazia, com único propósito de ser passada como parâmetro.
		*/
    void takeoffActiveCallback();

    //! Callback de feedback da ação takeoff
		/*!
		*   *	É executado quando a ação takeoff recebe feedback.
    *   *   Na implementação atual é uma função vazia, com único propósito de ser passada como parâmetro.
		*/
    void takeoffFeedbackCallback(
      const hector_uav_msgs::TakeoffFeedback::ConstPtr& feedback);

    //! Callback de resultado da ação pose
		/*!
		*   *	Recebe e trata o resultado da ação pose.
		*/
    void poseGoalCallback(const actionlib::SimpleClientGoalState& state,
      const hector_uav_msgs::PoseResult::ConstPtr& result);

    //! Callback de atividade da ação pose
		/*!
		*   *	É executado quando um objetivo da ação pose se torna ativo.
    *   *   Na implementação atual é uma função vazia, com único propósito de ser passada como parâmetro.
		*/
    void poseActiveCallback();

    //! Callback de feedback da ação pose
		/*!
		*   *	É executado quando a ação pose recebe feedback.
    *   *   Na implementação atual é uma função vazia, com único propósito de ser passada como parâmetro.
		*/
    void poseFeedbackCallback(
      const hector_uav_msgs::PoseFeedback::ConstPtr& feedback);

    //! Callback de resultado da ação landing
		/*!
		*   *	Recebe e trata o resultado da ação landing.
		*/
    void landingGoalCallback(const actionlib::SimpleClientGoalState& state,
      const hector_uav_msgs::LandingResult::ConstPtr& result);

    //! Callback de atividade da ação landing
		/*!
		*   *	É executado quando um objetivo da ação landing se torna ativo.
    *   *   Na implementação atual é uma função vazia, com único propósito de ser passada como parâmetro.
		*/
    void landingActiveCallback();

    //! Callback de feedback da ação landing
		/*!
		*   *	É executado quando a ação landing recebe feedback.
    *   *   Na implementação atual é uma função vazia, com único propósito de ser passada como parâmetro.
		*/
    void landingFeedbackCallback(
      const hector_uav_msgs::LandingFeedback::ConstPtr& feedback);

    //! Callback do serviço ScheduleWaypoint
		/*!
		*   *	É executado quando o serviço recebe uma requisição.
		*/
    bool scheduleWaypointCallback(
      drone_middleware::ScheduleWaypoint::Request  &req, 
      drone_middleware::ScheduleWaypoint::Response &res);
    
    //! Callback do serviço CancelWaypoints
		/*!
		*   *	É executado quando o serviço recebe uma requisição.
		*/
    bool cancelWaypointsCallback(
      drone_middleware::CancelWaypoints::Request  &req, 
      drone_middleware::CancelWaypoints::Response &res);
    
    //! Callback do serviço GetCurrentGoal
		/*!
		*   *	É executado quando o serviço recebe uma requisição.
		*/
    bool getCurrentGoalCallback(
      drone_middleware::GetCurrentGoal::Request  &req, 
      drone_middleware::GetCurrentGoal::Response &res);
    
    //! Callback do serviço SetMaximumAttempts
		/*!
		*   *	É executado quando o serviço recebe uma requisição.
		*/
    bool setMaximumAttemptsCallback(
      drone_middleware::SetMaximumAttempts::Request  &req, 
      drone_middleware::SetMaximumAttempts::Response &res);
    
    //! Callback do serviço SetTwistLimit
		/*!
		*   *	É executado quando o serviço recebe uma requisição.
		*/
    bool setTwistLimitCallback(
      drone_middleware::SetTwistLimit::Request  &req, 
      drone_middleware::SetTwistLimit::Response &res);

    public:

    //! Construtor da classe.
		/*!
		*   *	Possui muitos argumentos para permitir flexibilidade. Na falta de necessidade para tal basta usar os valores padrão.
    *   *   nodeName: nome do nó e prefixo dos tópicos/serviços etc associados a ele. (padrão: drone_waypoint_core)
    *   *   prefix: prefixo das ações, antes do número do drone. (padrão: drone)
    *   *   takeoffSuffix: nome da ação TakeoffAction. (padrão: action/takeoff)
    *   *   poseSuffix: nome da ação PoseAction. (padrão: action/pose)
    *   *   landingSuffix: nome da ação LandingAction. (padrão: action/landing)
    *   *   limitSuffix: nome do tópico usado para limitar velocidade. (padrão: "command/twist_limit")
    *   *   completedWaypointsTopic: nome do tópico em que estado dos Waypoints é publicado. (padrão: "waypoint_state")
    *   *   nodeName: nome desse módulo. (padrão: "drone_waypoint_core")
    *   *   scheduleWaypointService: nome do serviço ScheduleWaypoint. (padrão: schedule_waypoint) 
    *   *   cancelWaypointsService: nome do serviço CancelWaypoints. (padrão: cancel_waypoints)
    *   *   getCurrentGoalService: nome do serviço GetCurrentGoal. (padrão: get_current_goal)
    *   *   setMaximumAttemptsService: nome do serviço  SetMAximumAttempts. (padrão: set_maximum_attempts)
    *   *   setTwistLimitService: nome do serviço SetTwistLimit. (padrão: set_twist_limit)
    *   *   bufferSize: tamanho do buffer de mensagens nos tópicos. (padrão: 100)
    *   *   maximumFailedAttempts: valor inicial do máximo de falhas para um comando. (padrão: 3)
		*/
    CoreWaypoint(std::string prefix, std::string takeoffSuffix, std::string poseSuffix, 
      std::string landingSuffix, std::string limitSuffix, std::string nodeName, 
      std::string completedWaypointsTopic, std::string scheduleWaypointService, std::string cancelWaypointsService, 
      std::string getCurrentGoalService, std::string setMaximumAttemptsService, 
      std::string setTwistLimitService, uint16_t bufferSize, uint8_t maximumFailedAttempts);

    //! Configuração de número de drones
		/*!
		*   *	Chamar no para configurar número de drones a partir do servidor de parâmetros do ROS.
    *   *   Parâmetro padrão: /drone_middleware/n_drones
		*/ 
    static int setAndGetNDrones(std::string paramName);


    //! Configura o valor máximo de tentativas antes de marcar a ação como FAILED
		/*!
		*   *	É simplesmente uma função setter.
		*/
    void setMaximumAttempts(uint8_t newValue);

    //! Esperar os servidores de ações
		/*!
		*   *	Bloqueia e espera pelos servidores de ações do hector_quadrotor que serão utilizados.
		*/
    void waitForActionServers();

    //! Limpar e inicializar waypoints
		/*!
		*   *	Limpa e inicializa a estrutura de dados waypoints.
    *   *   Se all = 1 (true), limpa as filas de todos os drones.
    *   *   O retorno informa quantos comandos foram cancelados no total.
		*/
    uint16_t clearWaypointQueue(uint8_t all, uint16_t droneNumber);

    //! Cancelar os objetivos atuais de movimento de drones
		/*!
		*   *	Cancela todos os objetivos atuais de movimento do drone especificado.
    *   *   Se all = 1 (true), limpa para todos os drones.
		*   *   O retorno informa quantos comandos foram cancelados no total.
    */
    uint16_t cancelCurrentGoal(uint8_t all, uint16_t droneNumber);

    //! Interface utilizada para introduzir um comando de posição nas estruturas internas do módulo.
		/*!
		*   *	Essa função apenas coloca o comando na fila apropriada, o restante ocorre em métodos privados.
		*   *   O retorno é booleano e informa se adicionou corretamente ou não.
    */
    uint8_t scheduleWaypoint(drone_middleware::DroneWaypoint waypoint);

    //! Envia o limite de velocidade via tópico para um ou todos os drones.
		/*!
		*   *	O limite é informado como geometry_msgs::Twist.
    *   *   Se all = 1 (true), limita para todos os drones.
    * 	*	Para ver mais sobre geometry_msgs/Twist consulte: \<http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Twist.html\>
		*/
    void sendTwistLimit(uint8_t all, uint16_t droneNumber, geometry_msgs::Twist limit);

  };

//#endregion

}

#endif