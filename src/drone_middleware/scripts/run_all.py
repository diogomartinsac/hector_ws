#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rostopic
import rosservice
import rospkg
import re
import sys
import subprocess
import time
import os
import errno
import signal
import atexit
import math
import csv
from datetime import datetime

try:

    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()

    try:

        validArgs = [False]*len(sys.argv)
        validArgs[0] = True

        for n in range(0, len(sys.argv)):

            if sys.argv[n] == "--velcsvfile" or sys.argv[n] == "-vf":
                CSV_FILE = sys.argv[n+1]
                validArgs[n] = True
                validArgs[n+1] = True

            elif sys.argv[n] == "--poscsvfile" or sys.argv[n] == "-pf":
                POS_FILE = sys.argv[n+1]
                validArgs[n] = True
                validArgs[n+1] = True

            elif sys.argv[n] == "--drones" or sys.argv[n] == "-d":
                N_DRONES = int(sys.argv[n+1])
                validArgs[n] = True
                validArgs[n+1] = True
            
            elif sys.argv[n] == "--killall" or sys.argv[n] == "-ka":
                KILL_ALL = True
                validArgs[n] = True

            elif sys.argv[n] == "--nogui" or sys.argv[n] == "-ng":
                NO_GUI = True
                validArgs[n] = True
            
            elif sys.argv[n] == "--noterm" or sys.argv[n] == "-nt":
                NO_TERM = True
                validArgs[n] = True

            elif sys.argv[n] == "--time" or sys.argv[n] == "-t":
                TEST_TIME = int(sys.argv[n+1])
                validArgs[n] = True
                validArgs[n+1] = True

            elif sys.argv[n] == "--logusage" or sys.argv[n] == "-lu":
                LOG_USAGE = True
                validArgs[n] = True

            elif sys.argv[n] == "--killself" or sys.argv[n] == "-ks":
                KILL_SELF = True
                validArgs[n] = True

            elif sys.argv[n] == "--heatmap" or sys.argv[n] == "-hm":
                HEAT_MAP = True
                HEAT_MAP_X_MIN = float(sys.argv[n+1])
                HEAT_MAP_Y_MIN = float(sys.argv[n+2])
                HEAT_MAP_WIDTH = float(sys.argv[n+3])
                HEAT_MAP_CELLS = int(sys.argv[n+4])
                validArgs[n] = True
                validArgs[n+1] = True
                validArgs[n+2] = True
                validArgs[n+3] = True
                validArgs[n+4] = True


            elif sys.argv[n] == "--help" or sys.argv[n] == "-h":
                print(
    """List of arguments:
    --drones OR -d : Number of drones present in the simulation. (Default: 5)
    --heatmap OR -hm : Sets parameters for pose_listener to draw a heatmap. (Syntax: -hm x_min y_min width n_cells)
    --killall OR -ka : If used this script kills all instances of gzserver and roscore when it starts. (Default: False)
    --killself OR -ks : If false keeps everything open until manually closing this script, if true, closes itself after "time" steps of simulation. (This script kills all child processes at exit) (Default: False).
    --logusage OR -lu : Log the usage of CPU, RAM and disk. (Default: False)
    --nogui OR -ng : Uses gzserver instead of gazebo. Does not display 3D graphics, runs faster. (Default: False)
    --noterm OR -nt : Dumps output into log file instead of opening terminals. (Default: False)
    --poscsvfile OR -pf : Filename of the CSV file containing initial drone positions. (Default: Internal algorithm)
    --time OR -t : Amount of steps the simulation should run for when it starts. (Default: 0)
    --velcsvfile OR -vf : Filename of the CSV file containing drone movement commands. (Default: None)

    --help OR -h : Display this help message.""")
                sys.exit()
        
        invalidArgs = False
        for n in range(0, len(sys.argv)):
            if not validArgs[n]:
                print("Invalid argument: " + sys.argv[n])
                invalidArgs = True
        if invalidArgs:
            sys.exit()
            
    except (IndexError, ValueError):

        print("Error in the arguments passed")
        sys.exit()

    #variables
    PKG_NAME = "drone_middleware"

    PKG_PATH = rospack.get_path(PKG_NAME)

    CSV_PATH = PKG_PATH + "/csv"

    try:
        #CSV_FILE
        CSV_PARAM = CSV_PATH + "/" + CSV_FILE
    except NameError:
        #CSV_FILE = "drone_movement_template.csv"
        CSV_PARAM = None

    try:
        POS_FILE
    except NameError:
        POS_FILE = None

    try:
        N_DRONES
    except NameError:
        N_DRONES = 5

    LAUNCH_PATH = PKG_PATH + "/launch"

    LAUNCH_FILE = "spawn_" + str(N_DRONES) + "_quadrotors.launch"

    try:
        TEST_TIME
    except NameError:
        TEST_TIME=0

    TEMPLATE_FILE = "spawn_quadrotors_template.launch"

    try:
        KILL_ALL
    except NameError:
        KILL_ALL = False

    try:
        NO_GUI
    except NameError:
        NO_GUI = False

    try:
        NO_TERM
    except NameError:
        NO_TERM = False

    try:
        KILL_SELF
    except NameError:
        KILL_SELF = False

    try:
        LOG_USAGE
    except NameError:
        LOG_USAGE = False

    try:
        HEAT_MAP
    except NameError:
        HEAT_MAP = False

    LOG_PATH = PKG_PATH + "/logs"

    SCRIPT_PATH = PKG_PATH + "/scripts"

    if(NO_GUI):
        GAZEBO = "gzserver"
    else:
        GAZEBO = "gazebo"

    if NO_TERM:
        NOW = datetime.now()
        DT_STRING = NOW.strftime("%d-%m-%Y_%H-%M-%S")
        logfileName = LOG_PATH + "/" + DT_STRING + ".txt"
        try:
            os.makedirs(os.path.dirname(logfileName))
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise
        LOG_FILE = open(logfileName, "a", buffering=1)
        TERM = "noterm"
        PRINTABLE = "file"
        print("This run will use file " + LOG_PATH + "/" + DT_STRING + ".txt")
    else:
        TERM = "xterm"
        PRINTABLE = "cli"

    #list of child processes to be killed on exit
    child_list = []

    #run on xterm and hold the outputs
    def xterm_run(text):

        global child_list

        XTERM_HOLD = "xterm -hold -e "

        new_child = subprocess.Popen((XTERM_HOLD + text).split(" "))
        child_list.append(new_child)

        return new_child

    #run without xterm
    def noterm_run(text):

        global child_list
        global LOG_FILE

        BUFFERING = "stdbuf -oL -eL "

        LOG_FILE.write(text + '\n')

        new_child = subprocess.Popen((BUFFERING + text).split(" "), stdout=LOG_FILE, stderr=LOG_FILE)
        child_list.append(new_child)

        return new_child

    #encapsulate python print function
    def print_cli(string):

        print(string)

    #print to file
    def print_file(string):
        
        global LOG_FILE

        print(string)

        LOG_FILE.write(string + "\n")

    run = {
        "xterm": xterm_run,
        "noterm": noterm_run
    }

    print_ = {

        "cli" : print_cli,
        "file" : print_file
    }

    #kill all processes on child_list
    def kill_all_child_processes():

        global child_list

        for child in reversed(child_list):
            if not child.pid is None:
                #os.kill(child.pid, signal.SIGTERM)
                os.killpg(os.getpgid(child.pid), signal.SIGTERM)  # Send the signal to all the process groups

    #register to execute on exit
    atexit.register(kill_all_child_processes)

    if(KILL_ALL):
        run[TERM]("killall gzserver gazebo gazeb roscore xterm")
        time.sleep(1)

    #start roscore
    run[TERM]("roscore")
    #wait for master to be initialized
    looping = True
    while looping:
        try:
            rostopic.get_info_text("/rosout")
            looping = False

        except rostopic.ROSTopicException:
            time.sleep(0.1)

    print_[PRINTABLE]("ROS master started via roscore.")
    
    #setting parameters on ROS parameter server
    rospy.set_param("/" + PKG_NAME + "/n_drones", N_DRONES)
    print_[PRINTABLE]("Parameter /" + PKG_NAME + "/n_drones was set to " + str(N_DRONES) + ".")
    if not CSV_PARAM is None:
        rospy.set_param("/" + PKG_NAME + "/csv_path", CSV_PARAM)
        print_[PRINTABLE]("Parameter /" + PKG_NAME + "/csv_path was set to " + CSV_PARAM + ".")
    rospy.set_param("/" + PKG_NAME + "/nodes_online", 0)

    rospy.set_param("/" + PKG_NAME + "/heatmap/active", HEAT_MAP)
    print_[PRINTABLE]("Parameter /" + PKG_NAME + "/heatmap/active was set to " + str(HEAT_MAP) + ".")
    if(HEAT_MAP):
        rospy.set_param("/" + PKG_NAME + "/heatmap/x_min", HEAT_MAP_X_MIN)
        print_[PRINTABLE]("Parameter /" + PKG_NAME + "/heatmap/x_min was set to " + str(HEAT_MAP_X_MIN) + ".")
        rospy.set_param("/" + PKG_NAME + "/heatmap/y_min", HEAT_MAP_Y_MIN)
        print_[PRINTABLE]("Parameter /" + PKG_NAME + "/heatmap/y_min was set to " + str(HEAT_MAP_Y_MIN) + ".")
        rospy.set_param("/" + PKG_NAME + "/heatmap/width", HEAT_MAP_WIDTH)
        print_[PRINTABLE]("Parameter /" + PKG_NAME + "/heatmap/width was set to " + str(HEAT_MAP_WIDTH) + ".")
        rospy.set_param("/" + PKG_NAME + "/heatmap/n_cells", HEAT_MAP_CELLS)
        print_[PRINTABLE]("Parameter /" + PKG_NAME + "/heatmap/n_cells was set to " + str(HEAT_MAP_CELLS) + ".")

    #read launch template
    with open(LAUNCH_PATH + "/" + TEMPLATE_FILE, "r") as template:
        data = template.read()

    #if provided, read initial positions
    if not POS_FILE is None:
        with open(CSV_PATH + "/" + POS_FILE, "r") as posFile:
            positionList = list(csv.reader(posFile))[1:] #reads csv and ignores the first
            #print(positionList) #debug
            if len(positionList) < N_DRONES:
                print("Not enough values in initial position file: " + POS_FILE + ".")
                sys.exit()
            for line in positionList:
                if len(line) < 2:
                    print("Not enough values in initial position file: " + POS_FILE + ".")
                    sys.exit()
    
    #pattern to divide the file into 3 parts, the middle one needs to be replicated
    template_pattern = re.compile("(^.*?)(\n\t*<group.*?</group>\n)(.*?$)", re.DOTALL)
    template_extracted = re.search(template_pattern, data)

    #write the actual launch file that will be used, based on the template
    with open(LAUNCH_PATH + "/" + LAUNCH_FILE, "w") as launch:

        width = int(math.sqrt(N_DRONES))

        launch.write(template_extracted.group(1))

        for n in range(N_DRONES):
            
            if not POS_FILE is None:
                try:
                    droneX = float(positionList[n][0])
                    droneY = float(positionList[n][1])
                except ValueError:
                    print("Non-real value found in initial position file: " + POS_FILE + ".")
                    sys.exit()
            else:
                #define initial X and Y for each drone
                droneX = -1*(width/2) + (n/width)
                droneY = -1*(width/2) + (n%width)

            temp = template_extracted.group(2)
            temp = temp.replace("drone0", "drone" + str(n))
            temp = temp.replace("arg name=\"x\" value=\"0.0\"", "arg name=\"x\" value=\"" + str(droneX) + "\"")
            temp = temp.replace("arg name=\"y\" value=\"0.0\"", "arg name=\"y\" value=\"" + str(droneY) + "\"")

            launch.write(temp)

        launch.write(template_extracted.group(3))

    print_[PRINTABLE]("Template " + LAUNCH_PATH + "/" + LAUNCH_FILE + " was generated.")

    #starting gazebo
    run[TERM]("rosrun gazebo_ros " + GAZEBO)
    #wait for gazebo to be initialized
    looping = True
    while looping:
        try:
            rostopic.get_info_text("/clock")
            looping = False

        except rostopic.ROSTopicException:
            time.sleep(0.1)

    print_[PRINTABLE]("Gazebo started via rosrun.")

    #launching the .launch file that spawns drones
    run[TERM]("roslaunch drone_middleware " + LAUNCH_FILE)
    #waiting for all drones to be present
    #waiting for motor starter servers to be operational
    for n in range(N_DRONES):
        looping = True
        while looping:
            try:
                rostopic.get_info_text("/drone" + str(n) + "/cmd_vel")
                #rosservice.get_service_node("/drone" + str(n) + "/enable_motors")
                looping = False

            except rostopic.ROSTopicException:
                time.sleep(0.1)
    
    print_[PRINTABLE]("\rSpawned " + str(N_DRONES) + " drones via roslaunch.")
    
    #print("Dormindo")
    #while True:
        #time.sleep(1)

    NODES_ONLINE = rospy.get_param("/" + PKG_NAME + "/nodes_online")
    #starting drone_teleop_core
    run[TERM]("rosrun drone_middleware drone_teleop_core")
    print_[PRINTABLE]("drone_teleop_core started via rosrun.")
    #waiting for drone_teleop_core to be ready
    while NODES_ONLINE == rospy.get_param("/" + PKG_NAME + "/nodes_online"):
        time.sleep(0.1)

    NODES_ONLINE = rospy.get_param("/" + PKG_NAME + "/nodes_online")
    #starting drone_waypoint_core
    run[TERM]("rosrun drone_middleware drone_waypoint_core")
    print_[PRINTABLE]("drone_waypoint_core started via rosrun.")
    #waiting for drone_waypoint_core to be ready
    while NODES_ONLINE == rospy.get_param("/" + PKG_NAME + "/nodes_online"):
        time.sleep(0.1)

    NODES_ONLINE = rospy.get_param("/" + PKG_NAME + "/nodes_online")
    #starting gazebo_world_controller
    run[TERM]("rosrun drone_middleware gazebo_world_controller")
    print_[PRINTABLE]("gazebo_world_controller started via rosrun.")
    #waiting for gazebo_world_controller to be ready
    while NODES_ONLINE == rospy.get_param("/" + PKG_NAME + "/nodes_online"):
        time.sleep(0.1)

    if not CSV_PARAM is None:
        NODES_ONLINE = rospy.get_param("/" + PKG_NAME + "/nodes_online")
        #starting drone_teleop_csv
        run[TERM]("rosrun drone_middleware drone_teleop_csv")
        print_[PRINTABLE]("drone_teleop_csv started via rosrun.")
        #waiting for drone_teleop_csv to be ready
        while NODES_ONLINE == rospy.get_param("/" + PKG_NAME + "/nodes_online"):
            time.sleep(0.1)

    NODES_ONLINE = rospy.get_param("/" + PKG_NAME + "/nodes_online")
    #starting pose_listener
    run[TERM]("rosrun drone_middleware pose_listener")
    print_[PRINTABLE]("pose_listener started via rosrun.")
    #waiting for pose_listener to be ready
    while NODES_ONLINE == rospy.get_param("/" + PKG_NAME + "/nodes_online"):
        time.sleep(0.1)

    if TEST_TIME > 0:
        #running simulation for specific amount of time
        print_[PRINTABLE]("Sending goal of " + str(TEST_TIME) + " steps of simulation to Gazebo.")
        with open(os.devnull, 'w') as devnull:
            goal = subprocess.Popen(["stdbuf", "-oL", "-eL", "rostopic", "pub", "-1", "/gazebo_world_controller/multi_step/goal", "drone_middleware/MultiStepActionGoal", '{header: {seq: 0, stamp: 0, frame_id: ''}, goal_id: {stamp: 0, id: ''}, goal: {number: ' + str(TEST_TIME) + '}}'], stdout=devnull)
            child_list.append(goal)

    #log resource usage
    if LOG_USAGE:
        print_[PRINTABLE]("Starting to monitor resource usage.")
        TEMP_LOG = LOG_FILE
        logfileName = LOG_PATH + "/" + DT_STRING + "_usage.txt"
        try:
            os.makedirs(os.path.dirname(logfileName))
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise
        LOG_FILE = open(logfileName, "a", buffering=1)
        LOG_FILE.write(str(N_DRONES) + " drones\n")
        #run[TERM](SCRIPT_PATH + "/resource_usage.sh")
        run[TERM]("top -b -d 0.5 -u 1")
        LOG_FILE = TEMP_LOG

    if TEST_TIME > 0:
        #wait for simulation to start
        while rospy.get_param("/" + PKG_NAME + "/sim_stepping") == "false":
            time.sleep(0.1)
        #wait for simulation to end
        while rospy.get_param("/" + PKG_NAME + "/sim_stepping") == "true":
            time.sleep(1)
        #inform simulation completion
        print_[PRINTABLE]("Simulation of " + str(TEST_TIME) + " steps ended.")

    if not KILL_SELF:
        print("Press Ctrl+C to kill all xterm child processes")

        while True:
            time.sleep(60)

except KeyboardInterrupt:
    pass
    