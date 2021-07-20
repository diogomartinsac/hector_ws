#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Twist as TwistMsg
from drone_middleware.srv import Pause
from drone_middleware.srv import Unpause
from drone_middleware.srv import Restart
from drone_middleware.srv import GetSimInfo
from drone_middleware.srv import PoseRequest
from drone_middleware.srv import GetCurrentGoal
from drone_middleware.srv import SetMaximumAttempts
from drone_middleware.srv import SetTwistLimit
from drone_middleware.srv import CancelWaypoints
from drone_middleware.srv import ScheduleWaypoint
from drone_middleware.srv import ScheduleMovement
from drone_middleware.srv import GetHeatMapData
from drone_middleware.msg import WaypointState
from drone_middleware.msg import MultiStepAction
from drone_middleware.msg import MultiStepGoal
from actionlib_msgs.msg import GoalStatus

from drone_middleware.srv import PoseRequestRequest
from drone_middleware.srv import PauseRequest
from drone_middleware.srv import UnpauseRequest
from drone_middleware.srv import RestartRequest
from drone_middleware.srv import GetSimInfoRequest
from drone_middleware.srv import ScheduleMovementRequest
from drone_middleware.srv import CancelWaypointsRequest
from drone_middleware.srv import GetCurrentGoalRequest
from drone_middleware.srv import ScheduleWaypointRequest
from drone_middleware.srv import SetMaximumAttemptsRequest
from drone_middleware.srv import SetTwistLimitRequest
from drone_middleware.srv import GetHeatMapDataRequest
from drone_middleware.msg import MultiStepGoal

class DM_Handler(object):
    """
    Classe que facilita a utilização das interfaces do drone_middleware

    Atributos
    ----------
    *_SERVICE : str
        string que representa um serviço nos diretórios ROS

    *_TOPIC : str
        string que representa um tópico nos diretórios ROS

    *_ACTION : str
        string que representa uma ação nos diretórios ROS

    _currentSeqNum : int
        número de sequência atual, utilizado por schedule_NoSeq

    Métodos
    -------
    _blankWaypointCallback(self, message)
        função placeholder que chama um waypointCallback caso seja registrado um

    registerWaypointCallback(self, callback)
        registra uma função para ser chamada ao receber mensagens do tópico WAYPOINT_TOPIC

    registerStepCallback(self, callback)
        registra uma função para ser chamada ao receber resultados do stepGazebo

    waitForStepServer(self)
        aguarda que o servidor de ações esteja pronto para receber comandos para o Gazebo

    waitForServices(cls)
        aguarda que os serviços drone_middleware estejam prontos para receber solicitações

    stepGazebo(self, numberSteps)
        informa ao gazebo um valor em iterações de tempo que deve ser atingido

    pauseGazebo(cls)
        pausa o tempo no Gazebo
    
    unpauseGazebo(cls)
        faz o tempo correr no Gazebo caso esteja pausado
    
    restartGazebo(cls)
        reinicia o tempo no Gazebo

    getSimInfo(cls)
        solicita informações sobre a simulação no gazebo

    poseRequest(cls, all, droneNumber)
        solicita a pose atual do(s) drone(s)
    
    getHeatMap(cls)
        solicita os dados do heatmap atual
    
    getCurrentGoal(cls, droneNumber)
        solicita qual o objetivo de pose atual do drone e seu estado
    
    setMaximumAttempts(cls, maximumAttempts)
        define um máximo de tentativas para um mesmo comando de pose antes de ser considerado falho

    setTwistLimit(cls, all, droneNumber, limit)
        define uma velocidade máxima a ser desenvolvida pelo(s) drone(s)
    
    cancelWaypoints(cls, all, droneNumber, cancelCurrent, clearQueue)
        cancela objetivos de pose do(s) drone(s), podendo estar na fila e/ou atuais
    
    scheduleWaypoint(cls, droneNumber, seqNum, pose, takeoff, land, landAt)
        adiciona na fila de poses do drone selecionado um comando de pose
    
    scheduleMovement(cls, droneNumber, startAt, movement)
        adiciona na fila de velocidades dro drone selecionado um comando de velocidade
    
    schedule_NoSeq(cls, droneNumber, pose, takeoff, land, landAt)
        mesma função de scheduleWaypoint porem seqNum é definido pela classe automaticamente

    takeoff(cls, droneNumber)
        simplificação de schedule_NoSeq para decolar (iniciar movimento) de um drone
    
    landBelow(cls, droneNumber)
        simplificação de schedule_NoSeq para pousar um drone diretamente abaixo
    
    landWithPose(cls, droneNumber, pose)
        simplificação de schedule_NoSeq para pousar um drone numa pose específica
    
    setPose(cls, droneNumber, pose)
        simplificação de schedule_NoSeq para alterar a pose de um drone

    createPoseMsg(pX, pY, pZ, oX, oY, oZ, oW)
        retorna uma mensagem geometry_msgs/Pose do ROS
    
    createTwistMsg(lX, lY, lZ, aX, aY, aZ)
        retorna uma mensagem geometry_msgs/Twist do ROS

    stateString(code)
        retorna uma string que representa o estado do comando de pose (ACTIVE, SUCCEEDED, FAILED, CANCELED)
    
    waypointTypeString(waypointStateMsg)
        retorna uma string que representa o tipo do comando de pose (TAKEOFF, POSE, LAND, LANDAT)

    actionStateString(state)
        retorna uma string que representa o estado de uma ROS Action

    callByObject(self, obj)
        recebe um objeto das classes *Request do ROS e chama o método apropriado
    """

    PAUSE_SERVICE = "/gazebo_world_controller/pause"
    UNPAUSE_SERVICE = "/gazebo_world_controller/unpause"
    RESTART_SERVICE = "/gazebo_world_controller/restart"
    SIMINFO_SERVICE = "/gazebo_world_controller/get_sim_info"
    POSEREQUEST_SERVICE = "/pose_listener/pose_request"
    ATTEMPTS_SERVICE = "/drone_waypoint_core/set_maximum_attempts"
    TWISTLIMIT_SERVICE = "/drone_waypoint_core/set_twist_limit"
    CANCELWP_SERVICE = "/drone_waypoint_core/cancel_waypoints"
    WAYPOINT_SERVICE = "/drone_waypoint_core/schedule_waypoint"
    CURRENTGOAL_SERVICE = "/drone_waypoint_core/get_current_goal"
    MOVEMENT_SERVICE = "/drone_teleop_core/schedule_movement"
    HEATMAP_SERVICE = "/pose_listener/get_heatmap_data"

    WAYPOINT_TOPIC = "/drone_waypoint_core/waypoint_state"

    STEP_ACTION = "/gazebo_world_controller/multi_step"

    _currentSeqNum = 0

    def __init__(self, waypointCallback=None, stepCallback=None):
        """
        Parâmetros
        ----------
        waypointCallback : function
            função que será chamada quando receber mensagens do tópico WAYPOINT_TOPIC
        stepCallback : function
            função que será chamada quando receber resultados de stepGazebo
        """

        self.waypointCallback = waypointCallback
        self.stepCallback = stepCallback

        self.waypointSubscriber = rospy.Subscriber(self.WAYPOINT_TOPIC, WaypointState, self._blankWaypointCallback)

        self.stepClient = actionlib.SimpleActionClient(self.STEP_ACTION, MultiStepAction)

    def _blankWaypointCallback(self, message):
        """
        Parâmetros
        ----------
        message : WaypointState
            mensagem que será passada para o waypointCallback
        """

        if not self.waypointCallback is None:
            return self.waypointCallback(message)

    def registerWaypointCallback(self, callback):
        """
        Parâmetros
        ----------
        callback : function
            função que será chamada quando receber mensagens do tópico WAYPOINT_TOPIC
        """

        self.waypointCallback = callback

    def registerStepCallback(self, callback):
        """
        Parâmetros
        ----------
        callback : function
            função que será chamada quando receber resultados de stepGazebo
        """

        self.stepCallback = callback

    def waitForStepServer(self):
        self.stepClient.wait_for_server()

    @classmethod
    def waitForServices(cls):
        rospy.wait_for_service(cls.PAUSE_SERVICE)
        rospy.wait_for_service(cls.UNPAUSE_SERVICE)
        rospy.wait_for_service(cls.RESTART_SERVICE)
        rospy.wait_for_service(cls.SIMINFO_SERVICE)
        rospy.wait_for_service(cls.POSEREQUEST_SERVICE)
        rospy.wait_for_service(cls.CURRENTGOAL_SERVICE)
        rospy.wait_for_service(cls.ATTEMPTS_SERVICE)
        rospy.wait_for_service(cls.TWISTLIMIT_SERVICE)
        rospy.wait_for_service(cls.CANCELWP_SERVICE)
        rospy.wait_for_service(cls.UNPAUSE_SERVICE)
        rospy.wait_for_service(cls.MOVEMENT_SERVICE)

    def stepGazebo(self, numberSteps):
        """
        Parâmetros
        ----------
        numberSteps : function
            número final de iterações que deve ser atingido pelo gazebo
        """

        goal = MultiStepGoal(number=numberSteps)
        self.stepClient.send_goal(goal, done_cb=self.stepCallback)

    @classmethod
    def pauseGazebo(cls):
        try:
            pauseProxy = rospy.ServiceProxy(cls.PAUSE_SERVICE, Pause)
            response = pauseProxy()
            return response
        except rospy.ServiceException as e:
            print("Service call failed: " + str(e))

    @classmethod
    def unpauseGazebo(cls):
        try:
            unpauseProxy = rospy.ServiceProxy(cls.UNPAUSE_SERVICE, Unpause)
            response = unpauseProxy()
            return response
        except rospy.ServiceException as e:
            print("Service call failed: " + str(e))

    @classmethod
    def restartGazebo(cls):
        try:
            restartProxy = rospy.ServiceProxy(cls.RESTART_SERVICE, Restart)
            response = restartProxy()
            return response
        except rospy.ServiceException as e:
            print("Service call failed: " + str(e))

    @classmethod
    def getSimInfo(cls):
        """
        Retorna
        ----------
        response
            ver GetSimInfo.srv
        """

        try:
            simInfoProxy = rospy.ServiceProxy(cls.SIMINFO_SERVICE, GetSimInfo)
            response = simInfoProxy()
            return response
        except rospy.ServiceException as e:
            print("Service call failed: " + str(e))

    @classmethod
    def poseRequest(cls, all, droneNumber):
        """
        Parâmetros
        ----------
        all : int
            é usado como booleano, se 1 (True), requisita todos os drones
        droneNumber : int
            se all for 0 (False), requisita apenas o drone informado

        Retorna
        ----------
        response
            ver PoseRequest.srv
        """

        try:
            poseRequestProxy = rospy.ServiceProxy(cls.POSEREQUEST_SERVICE, PoseRequest)
            response = poseRequestProxy(all, droneNumber)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: " + str(e))

    @classmethod
    def getHeatMap(cls):
        """
        Retorna
        ----------
        response
            ver GetHeatMapData.srv
        """

        try:
            getHeatMapProxy = rospy.ServiceProxy(cls.HEATMAP_SERVICE, GetHeatMapData)
            response = getHeatMapProxy()
            return response
        except rospy.ServiceException as e:
            print("Service call failed: " + str(e))

    @classmethod
    def getCurrentGoal(cls, droneNumber):
        """
        Parâmetros
        ----------
        droneNumber : int
            número do drone a ser requisitado
            
        Retorna
        ----------
        response
            ver GetCurrentGoal.srv
        """

        try:
            currentGoalProxy = rospy.ServiceProxy(cls.CURRENTGOAL_SERVICE, GetCurrentGoal)
            response = currentGoalProxy(droneNumber)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: " + str(e))

    @classmethod
    def setMaximumAttempts(cls, maximumAttempts):
        """
        Parâmetros
        ----------
        maximumAttempts : int
            número máximo de vezes que um comando de pose será executado com falha antes de parar
        """

        try:
            maximumAttemptsProxy = rospy.ServiceProxy(cls.ATTEMPTS_SERVICE, SetMaximumAttempts)
            response = maximumAttemptsProxy(maximumAttempts)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: " + str(e))

    @classmethod
    def setTwistLimit(cls, all, droneNumber, limit):
        """
        Parâmetros
        ----------
        all : int
            é usado como booleano, se 1 (True), requisita a todos os drones
        droneNumber : int
            se all for 0 (False), requisita apenas ao drone informado
        limit : geometry_msgs/Twist
            objeto Twist que representa velocidades máximas lineares e angulares em seus eixos
        """

        try:
            twistLimitProxy = rospy.ServiceProxy(cls.TWISTLIMIT_SERVICE, SetTwistLimit)
            response = twistLimitProxy(all, droneNumber, limit)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: " + str(e))

    @classmethod
    def cancelWaypoints(cls, all, droneNumber, cancelCurrent, clearQueue):
        """
        Parâmetros
        ----------
        all : int
            é usado como booleano, se 1 (True), requisita a todos os drones
        droneNumber : int
            se all for 0 (False), requisita apenas ao drone informado
        cancelCurrent : int
            é usado como booleano, se 1 (True), cancela objetivo atual
        clearQueue : int
            é usado como booleano, se 1 (True), cancela objetivos na fila
            
        Retorna
        ----------
        response
            ver CancelWaypoints.srv
        """

        try:
            cancelWPProxy = rospy.ServiceProxy(cls.CANCELWP_SERVICE, CancelWaypoints)
            response = cancelWPProxy(all, droneNumber, cancelCurrent, clearQueue)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: " + str(e))

    @classmethod
    def scheduleWaypoint(cls, droneNumber, seqNum, pose, takeoff, land, landAt):
        """
        Parâmetros
        ----------
        droneNumber : int
            número do drone a ser requisitado
        seqNum : int
            número que define a ordem que os comandos serão realizados, menores números primeiro,
            se zero for informado o comando será colocado ao final da fila que existir quando chegar
            e esse valor zero será alterado para um número não especificado
        pose : geometry_msgs/Pose
            pose que o drone deve atingir (localização + orientação)
        takeoff : int
            é usado como booleano, se 1 (True), é um comando de decolagem
            comando de decolagem apenas prepara o drone para se mover
            esse comando ignora pose
        land : int
            é usado como booleano, se 1 (True), é comando de pouso
            se landAt for 1 (True), considera pose
            se landAt for 0 (False), pousa diretamente abaixo, desconsiderando pose
        landAt: int
            é usado como booleano, se 1 (True), considera pose no pouso

        Um comando não pode ser do tipo TAKEOFF e LAND ao mesmo tempo
        Se o comando não tiver TAKEOFF, LAND ou LANDAT, ele será POSE
        Comando só pode ter LANDAT se tiver LAND
        Combinações de TAKEOFF, LAND e LANDAT que não fazem sentido são inválidas
            
        Retorna
        ----------
        response
            ver scheduleWaypoint.srv
        """

        try:
            waypointProxy = rospy.ServiceProxy(cls.WAYPOINT_SERVICE, ScheduleWaypoint)
            response = waypointProxy(droneNumber, seqNum, pose, takeoff, land, landAt)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: " + str(e))

    @classmethod
    def scheduleMovement(cls, droneNumber, startAt, movement):
        """
        Parâmetros
        ----------
        droneNumber : int
            drone a ser requisitado
        startAt : std_msgs/Time
            tempo que o movimento deve iniciar
            o movimento continua até ser alterado ou indefinidamente caso contrário
        movement : geometry_msgs/Twist
            objeto Twist representando as velocidades lineares e angulares a serem desenvolvidas

        Retorna
        ----------
        response
            ver ScheduleMovement.srv
        """

        try:
            movementProxy = rospy.ServiceProxy(cls.MOVEMENT_SERVICE, ScheduleMovement)
            response = movementProxy(droneNumber, startAt, movement)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: " + str(e))

    @classmethod
    def schedule_NoSeq(cls, droneNumber, pose, takeoff, land, landAt):
        """
        Parâmetros
        ----------
        droneNumber : int
            número do drone a ser requisitado
        pose : geometry_msgs/Pose
            pose que o drone deve atingir (localização + orientação)
        takeoff : int
            é usado como booleano, se 1 (True), é um comando de decolagem
            comando de decolagem apenas prepara o drone para se mover
            esse comando ignora pose
        land : int
            é usado como booleano, se 1 (True), é comando de pouso
            se landAt for 1 (True), considera pose
            se landAt for 0 (False), pousa diretamente abaixo, desconsiderando pose
        landAt: int
            é usado como booleano, se 1 (True), considera pose no pouso

        Um comando não pode ser do tipo TAKEOFF e LAND ao mesmo tempo
        Se o comando não tiver TAKEOFF, LAND ou LANDAT, ele será POSE
        Comando só pode ter LANDAT se tiver LAND
        Combinações de TAKEOFF, LAND e LANDAT que não fazem sentido são inválidas
            
        Retorna
        ----------
        response
            ver scheduleWaypoint.srv
        """

        cls._currentSeqNum += 1
        return cls.scheduleWaypoint(droneNumber, cls._currentSeqNum, pose, takeoff, land, landAt)

    @classmethod
    def takeoff(cls, droneNumber):
        """
        Parâmetros
        ----------
        droneNumber : int
            número do drone a ser requisitado
            
        Retorna
        ----------
        response
            ver scheduleWaypoint.srv
        """

        return cls.schedule_NoSeq(droneNumber, PoseMsg(), True, False, False)

    @classmethod
    def landBelow(cls, droneNumber):
        """
        Parâmetros
        ----------
        droneNumber : int
            número do drone a ser requisitado
            
        Retorna
        ----------
        response
            ver scheduleWaypoint.srv
        """

        return cls.schedule_NoSeq(droneNumber, PoseMsg(), False, True, False)

    @classmethod
    def landWithPose(cls, droneNumber, pose):
        """
        Parâmetros
        ----------
        droneNumber : int
            número do drone a ser requisitado
        pose : geometry_msgs/Pose
            pose que o drone deve atingir (localização + orientação)
            
        Retorna
        ----------
        response
            ver scheduleWaypoint.srv
        """

        return cls.schedule_NoSeq(droneNumber, pose, False, True, True)

    @classmethod
    def setPose(cls, droneNumber, pose):
        """
        Parâmetros
        ----------
        droneNumber : int
            número do drone a ser requisitado
        pose : geometry_msgs/Pose
            pose que o drone deve atingir (localização + orientação)
            
        Retorna
        ----------
        response
            ver scheduleWaypoint.srv
        """

        return cls.schedule_NoSeq(droneNumber, pose, False, False, False)
    
    @staticmethod
    def createPoseMsg(pX, pY, pZ, oX, oY, oZ, oW):
        """
        Parâmetros
        ----------
        pX : float
            valor no eixo x da posição
        pY : float
            valor no eixo y da posição
        pZ : float
            valor no eixo z da posição
        oX : float
            valor no eixo x da orientação
        oY : float
            valor no eixo y da orientação
        oZ : float
            valor no eixo z da orientação
        oW : float
            valor no eixo w da orientação

        Orientação é dada em forma de quaternion (4 eixos em vez de 3)
            
        Retorna
        ----------
        msg : geometry_msgs/Pose
            mensagem do tipo geometry_msgs/Pose do ROS
        """

        msg = PoseMsg()
        msg.position.x = pX
        msg.position.y = pY
        msg.position.z = pZ
        msg.orientation.x = oX
        msg.orientation.y = oY
        msg.orientation.z = oZ
        msg.orientation.w = oW
        return msg

    @staticmethod
    def createTwistMsg(lX, lY, lZ, aX, aY, aZ):
        """
        Parâmetros
        ----------
        lX : float
            velocidade linear no eixo x
        lY : float
            velocidade linear no eixo y
        lZ : float
            velocidade linear no eixo z
        aX : float
            velocidade angular no eixo x
        aY : float
            velocidade angular no eixo y
        aZ : float
            velocidade angular no eixo z
            
        Retorna
        ----------
        msg : geometry_msgs/Twist
            mensagem do tipo geometry_msgs/Twist do ROS
        """
        
        msg = TwistMsg()
        msg.linear.x = lX
        msg.linear.y = lY
        msg.linear.z = lZ
        msg.angular.x = aX
        msg.angular.y = aY
        msg.angular.z = aZ
        return msg

    @staticmethod
    def stateString(code):
        """
        Esta função ilustra os códigos de estado dos comandos de pose
        ACTIVE significa que o comando foi iniciado e está em andamento
        SUCCEEDED significa que o comando foi executado com sucesso
        FAILED significa que o comando falhou mais do que o número aceitável de vezes
        CANCELED significa que o comando foi cancelado pelo usuário

        Se um comando receber o estado FAILED, aquele drone não executará mais comandos,
        ficará parado até que receba um cancelamento, após esse comando FAILED receber
        CANCELED, o drone pode voltar a executar comandos

        Parâmetros
        ----------
        code : int
            código que veio na mensagem WaypointState
            
        Retorna
        ----------
        str
            String representativa
        """

        code &= 0xff
        if code == (0 & 0xff):
            return "ACTIVE"
        elif code == (1  & 0xff):
            return "SUCCEEDED"
        elif code == (-1 & 0xff):
            return "FAILED"
        elif code == (-2 & 0xff):
            return "CANCELED"

    @staticmethod
    def waypointTypeString(waypointStateMsg):
        """
        Esta função ilustra os tipos de comando de pose
        POSE é comando para alterar posição e orientação
        TAKEOFF é comando de decolar (preparar para iniciar movimentos)
        LAND é comando de pouso diretamente abaixo
        LANDAT é comando de pouso com pose definida

        Parâmetros
        ----------
        waypointStateMsg : drone_middleware/WaypointState
            mensagem do tipo WaypointState que chegou no tópico WAYPOINT_TOPIC
            
        Retorna
        ----------
        str
            String representativa
        """

        if not waypointStateMsg.takeoff and not waypointStateMsg.land and not waypointStateMsg.landAt:
            return "POSE"
        elif waypointStateMsg.takeoff and not waypointStateMsg.land and not waypointStateMsg.landAt:
            return "TAKEOFF"
        elif waypointStateMsg.land and not waypointStateMsg.landAt and not waypointStateMsg.takeoff:
            return "LAND"
        elif waypointStateMsg.land and waypointStateMsg.landAt and not waypointStateMsg.takeoff:
            return "LANDAT"
        else:
            return "INVALID"

    @staticmethod
    def actionStateString(state):
        """
        Esta função ilustra os tipos de estado de ROS Actions
        Uma ação ROS pode estar em qualquer dos estados abaixo
        Para mais informações ver a wiki do ROS

        Parâmetros
        ----------
        state : actionlib_msgs/GoalStatus
            mensagem do tipo actionlib_msgs/GoalStatus que chegou de uma ROS Action
            
        Retorna
        ----------
        str
            String representativa
        """

        actionStates = {
            GoalStatus.PENDING: "PENDING",
            GoalStatus.ACTIVE: "ACTIVE",
            GoalStatus.PREEMPTED: "PREEMPTED",
            GoalStatus.SUCCEEDED: "SUCCEEDED",
            GoalStatus.ABORTED: "ABORTED",
            GoalStatus.REJECTED: "REJECTED",
            GoalStatus.PREEMPTING: "PREEMPTING",
            GoalStatus.RECALLING: "RECALLING",
            GoalStatus.RECALLED: "RECALLED",
            GoalStatus.LOST: "LOST"
        }

        return actionStates[state]

    def callByObject(self, obj):
        """
        Esta função recebe um objeto do tipo Request de serviço do ROS
        e chama a função apropriada para ele do DM_Handler com uso de
        lambda functions

        Parâmetros
        ----------
        obj : *
            objeto que seja de algum dos tipos válidos abaixo
            
        Retorna
        ----------
        response : *
            varia com o objeto informado
        """
        
        _call = {
        PoseRequestRequest: lambda x: self.poseRequest(x.all, x.droneNumber),
        PauseRequest: lambda x: self.pauseGazebo(),
        UnpauseRequest: lambda x: self.unpauseGazebo(),
        RestartRequest: lambda x: self.restartGazebo(),
        MultiStepGoal: lambda x: self.stepGazebo(x.number),
        GetSimInfoRequest: lambda x: self.getSimInfo(),
        ScheduleMovementRequest: lambda x: self.scheduleMovement(x.droneNumber, x.startAt, x.movement),
        CancelWaypointsRequest: lambda x: self.cancelWaypoints(x.all, x.droneNumber, x.cancelCurrent, x.clearQueue),
        GetCurrentGoalRequest: lambda x: self.getCurrentGoal(x.droneNumber),
        ScheduleWaypointRequest: lambda x: self.scheduleWaypoint(x.droneNumber, x.seqNum, x.pose, x.takeoff, x.land, x.landAt),
        SetMaximumAttemptsRequest: lambda x: self.setMaximumAttempts(x.maximumAttempts),
        SetTwistLimitRequest: lambda x: self.setTwistLimit(x.all, x.droneNumber, x.limit),
        GetHeatMapDataRequest: lambda x: self.getHeatMap()
        }

        return _call[type(obj)](obj)

#demo of API use
if __name__ == "__main__":

    from random import randint
    from time import sleep

    try:
        stillRunning = True

        def myWaypointCallback(message):
            """
            Exemplo de função criada pelo usuário para receber atualizações sobre os comandos de pose

            Parâmetros
            ----------
            message : drone_middleware/WaypointState
                contém todos os dados acerca do comando e seu estado
                para mais informações ver WaypointState.msg

            Aqui o usuário pode checar os valores de message para determinar qual comando está sendo informado
            e também qual estado foi atualizado para esse comando

            Por exemplo se o interesse for os comandos completados com sucesso basta checar se actionState
            corresponde ao código para sucesso e por aí vai
            """

            global stillRunning

            print(DM_Handler.waypointTypeString(message) + " command " + str(message.seqNum) + " has state: " + DM_Handler.stateString(message.actionState) + ".")

            if message.seqNum == 39 and DM_Handler.stateString(message.actionState) == "SUCCEEDED":
                stillRunning = False

        def myStepCallback(state, result):
            """
            Exemplo de função criada pelo usuário para receber resultados do comando stepGazebo

            Parâmetros
            ----------
            state : actionlib_msgs/GoalStatus
                estado que a ação recebeu
            result : drone_middleware/MultiStepResult
                resultado enviado pelo servidor de ações
                para mais informações ver MultiStep.action

            Aqui usuário pode receber confirmações de que o comando que ele deu de passos discretos no tempo
            para o Gazebo já terminou
            """

            global stepping

            print(str(result.completed) + " iterations completed with state: " + DM_Handler.actionStateString(state))
            stepping = False


        print("Starting demo.")

        steps = 0

        rospy.init_node("dm_demo_node", anonymous=True)

        dm = DM_Handler(waypointCallback=myWaypointCallback, stepCallback=myStepCallback)

        dm.waitForServices()
        dm.waitForStepServer()

        dm.takeoff(0)
        dm.setPose(0, dm.createPoseMsg(0, 0, 3, 0, 0, 0, 0))
        for i in range(0, 6):
            for j in range(0, 6):
                dm.setPose(0, dm.createPoseMsg(i*2-6, j*2-6, 3, 0, 0, 0, 0))
        dm.landBelow(0)
        dm.setTwistLimit(True, 0, dm.createTwistMsg(1, 1, 1, 1, 1, 1))
        dm.setMaximumAttempts(3)

        steps += randint(5000, 10000)
        print("Sendind Gazebo a random amount of steps.")
        stepping = True
        dm.stepGazebo(steps)

        while(stillRunning):
            sleep(0.5)
            if not stepping:
                sim = dm.getSimInfo()
                print("Time simulated so far: " + str(sim.simTime.secs + float(sim.simTime.nsecs) / 1000000000) + " seconds.")
                steps += randint(5000, 10000)
                print("Sendind Gazebo a random amount of steps.")
                stepping = True
                dm.stepGazebo(steps)

        print("Simulation of 39 waypoints ended")

    except rospy.exceptions.ROSInterruptException as e1:
        print(e1)
    
    except KeyboardInterrupt as e2:
        print("Interrupted by the user.")