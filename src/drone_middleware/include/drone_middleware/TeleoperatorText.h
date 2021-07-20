#ifndef CLASS_TELEOPERATORTEXT_H
#define CLASS_TELEOPERATORTEXT_H

//#region [include]

#include "drone_middleware/Teleoperator.h" 

#include <sstream>

//#endregion

//#region [class]

namespace drone_middleware
{   

  //! Classe de operação por texto.
  /*!
  *   *   Herdeira de Teleoperator.
  *   *	Essa classe permite efetuar testes controlando os drones via linha de comando
  */
  class TeleoperatorText : public Teleoperator
  {

    public:

    //! Construtor da classe.
    /*!
    *   *	Possui muitos argumentos para permitir flexibilidade. Na falta de necessidade para tal basta usar os valores padrão.
    *   *   serverName: nome do nó servidor e prefixo dos tópicos/serviços etc associados a ele. (padrão: drone_teleop_core)
    *   *   serviceName: nome do serviço utilizado. (padrão: schedule_movement)
    */
    TeleoperatorText(std::string serverName, std::string serviceName) :
      Teleoperator(serverName, serviceName) {};

    //! Função de imprimir forma de uso.
    /*!
    *   *	Simplesmente imprime instruções de uso na tela.
    */
    void displayUsageInfo();

    //! Função abstrata de adquirir entrada implementada.
    /*!
    *   *	A sua implementação deve adquirir uma entrada de movimento de alguma maneira e montar uma mensagem a ser enviada para o Core.
    *   *   Nesse caso a entrada é tomada da entrada padrão, via teclado.
    */
    bool getOneInput(drone_middleware::ScheduleMovement &srv_msg);

    //! Função abstrata de iniciar loop implementada.
    /*!
    *   *	A sua implementação inicia um loop que envolve adquirir entradas, enviá-las ao Core e atualizar o ROS.
    */
    int enterLoop();

  };

//#endregion

}

#endif