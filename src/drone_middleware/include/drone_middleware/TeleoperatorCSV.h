#ifndef CLASS_TELEOPERATORCSV_H
#define CLASS_TELEOPERATORCSV_H

//#region [include]

#include "drone_middleware/Teleoperator.h" 

#include <sstream>
#include <string>
#include <iostream>
#include <fstream>
#include <queue>

//#endregion

//#region [class]

namespace drone_middleware
{   

  //! Classe de operação por arquivo.
  /*!
  *   *   Herdeira de Teleoperator.
  *   *	Essa classe permite efetuar controle dos drones via arquivo previamente codificado em CSV.
  */
  class TeleoperatorCSV : public Teleoperator
  {

    private:

    //! Caminho do arquivo.
    /*!
    *   *	Pode ser obtido via servidor de parâmetros, argumento ou entrada padrão.
    */
    std::string path;

    //! Objeto do arquivo.
    /*!
    *   *	Simboliza o arquivo aberto.
    */
    std::ifstream csv_file;

    public:

    //! Construtor da classe.
    /*!
    *   *	Possui muitos argumentos para permitir flexibilidade. Na falta de necessidade para tal basta usar os valores padrão.
    *   *   serverName: nome do nó servidor e prefixo dos tópicos/serviços etc associados a ele. (padrão: drone_teleop_core)
    *   *   serviceName: nome do serviço utilizado. (padrão: schedule_movement)
    *   *   path: caminho do arquivo. (padrão: obter com getPath)
    */
    TeleoperatorCSV(std::string serverName, std::string serviceName, std::string path);

    //! Checa se a leitura do arquivo pode continuar.
    /*!
    *   *	Usado na manipulação do arquivo, checar EOF por exemplo.
    */
    bool checkCSVGood();

    //! Função abstrata de adquirir entrada implementada.
    /*!
    *   *	A sua implementação deve adquirir uma entrada de movimento de alguma maneira e montar uma mensagem a ser enviada para o Core.
    *   *   Nesse caso a entrada é tomada do arquivo CSV, linha por linha.
    */
    bool getOneInput(drone_middleware::ScheduleMovement &srv_msg);

    //! Função abstrata de iniciar loop implementada.
    /*!
    *   *	A sua implementação inicia um loop que envolve adquirir entradas, enviá-las ao Core e atualizar o ROS.
    */
    int enterLoop();

    //! Faz a leitura do CSV.
    /*!
    *   *	Lê o arquivo e consome seus dados para dentro do programa.
    */
    void consumeCSV();

    //! Função estática que determina caminho do arquivo CSV.
    /*!
    *   *   argc e argv: passar os mesmos parâmetros do main.
    *   *	paramName: parâmetro que indica o caminho do arquivo. (padrão: /drone_middleware/csv_path)
    */
    static std::string getPath(int argc, char **argv, std::string paramName);

  };

//#endregion

}

#endif