#ifndef CLASS_TELEOPERATOR_H
#define CLASS_TELEOPERATOR_H

//#region [include]

#include "ros/ros.h"

#include "drone_middleware/ScheduleMovement.h"

//#endregion

//#region [class]

namespace drone_middleware
{
  //! Classe abstrata que representa os teleoperadores em geral.
  /*!
  *   *	Essa classe representa os teleoperadores que se comunicam com o Core.
  *   *   A partir dela podem ser criadas outras formas de ditar o movimento dos drones.
  */
  class Teleoperator
  {
    
    private: 

    //! ROS Node Handle.
    /*!
    *   *	"Alça" pela qual o objeto se conecta ao ROS.
    *   *   NodeHandle deve ser criada apenas após o ros::init.
    */
    ros::NodeHandle n;
    
    //! Cliente do serviço de escalonar movimentos.
    /*!
    *   *	Cliente para chamar serviço de escalonar movimentos que será atendido pelo Core.
    */
    ros::ServiceClient movementSchedulerClient;

    public:

    //! Construtor da classe.
    /*!
    *   *	Possui muitos argumentos para permitir flexibilidade. Na falta de necessidade para tal basta usar os valores padrão.
    *   *   serverName: nome do nó servidor e prefixo dos tópicos/serviços etc associados a ele. (padrão: drone_teleop_core)
    *   *   serviceName: nome do serviço utilizado. (padrão: schedule_movement)
    */
    Teleoperator(std::string serverName, std::string serviceName);

    //! Função de chamada de serviço.
    /*!
    *   *	Chama o core para atender à solicitação e escalonar o movimento enviado.
    */
    int callSchedulerServer(drone_middleware::ScheduleMovement srv_msg);

    //! Função abstrata de adquirir entrada.
    /*!
    *   *	A sua implementação deve adquirir uma entrada de movimento de alguma maneira e montar uma mensagem a ser enviada para o Core.
    */
    virtual bool getOneInput(drone_middleware::ScheduleMovement &srv_msg) = 0;

    //! Função abstrata de iniciar loop.
    /*!
    *   *	A sua implementação inicia um loop que envolve adquirir entradas, enviá-las ao Core e atualizar o ROS.
    */
    virtual int enterLoop() = 0;
  };

//#endregion

}

#endif