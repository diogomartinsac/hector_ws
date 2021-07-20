#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from random import randint
from random import random
from copy import deepcopy
import math
import rospy
from dm_handler import DM_Handler
import sys
from datetime import datetime
import rospkg
import os
import errno

class MPL_Heatmap(object):

    def __init__(self, number_cells, area_width=None, x_min=None, y_min=None, cmap='YlGn', interpolation='none'):

        self.area_width = area_width
        self.number_cells = number_cells

        self.x_min = x_min
        self.y_min = y_min

        self.grid = []

        self.cmap = cmap
        self.interpolation = interpolation

        if not area_width is None:
            self.mul_factor = (number_cells/area_width)

    def clearGrid(self):
        self.grid = []
        for _ in range(self.number_cells):
            line = []
            for __ in range(self.number_cells):
                line.append(0)
            self.grid.append(line)
    
    def randomGrid(self, max):
        self.grid = []
        for _ in range(self.number_cells):
            line = []
            for __ in range(self.number_cells):
                line.append(randint(0, max))
            self.grid.append(line)

    def sparsePointsGrid(self, chance, max):
        self.grid = []
        for _ in range(self.number_cells):
            line = []
            for __ in range(self.number_cells):
                if random() <= chance:
                    line.append(randint(0, max))
                else:
                    line.append(0)
            self.grid.append(line)

    def gridFromArray(self, array):
        index = 0
        self.grid = []
        for _ in range(self.number_cells):
            line = []
            for __ in range(self.number_cells):
                line.append(array[index])
                index += 1
            self.grid.append(line)

    def getGridMaxValue(self):
        if len(self.grid) > 0:
            maxValue = self.grid[0][0]
            for line in self.grid:
                for value in line:
                    if value > maxValue:
                        maxValue = value
            return maxValue
        else:
            return None
    
    def setValue(self, i, j, value):
        self.grid[i][j] = value

    def incrementValue(self, i, j, value):
        self.grid[i][j] += value

    def setGrid(self, new_grid):
        self.grid = deepcopy(new_grid)

    def getCellForPoint(self, x, y):
        #deslocar o eixo de referencia para que o canto superior esquerdo da area resida na origem
        #new_x -= self.x_min
        #new_y -= self.y_min
        #i = floor(new_x * (number_cells/area_width))
        #j = floor(new_y * (number_cells/area_width))
        j = int(math.floor((x - self.x_min) * self.mul_factor))
        i = int(math.floor((y - self.y_min) * self.mul_factor))
        return i, j

    def incrementCellFromPoint(self, x, y):
        i, j = self.getCellForPoint(x, y)
        self.incrementValue(i, j, 1)

    def incrementCellFromList(self, point_list):
        last_cell = None
        for point in point_list:
            current_cell = self.getCellForPoint(point[0], point[1])
            if current_cell != last_cell:
                last_cell = current_cell
                self.incrementCellFromPoint(point[0], point[1])

    def generate(self):
        #Plot heatmap
        self.im = plt.imshow(self.grid, cmap=self.cmap, aspect='equal', origin='upper', extent=(-0.5, self.number_cells-0.5, -0.5, self.number_cells-0.5), interpolation=self.interpolation)

        # Plot a colorbar with label.
        self.cb = plt.colorbar()
        #self.cb.set_label('Number of cell traversals')
        self.cb.set_ticks(range(self.getGridMaxValue()+1))

        # Add title and labels to plot.
        #plt.title('Mobility Heatmap')
        #plt.xlabel('x axis')
        #plt.ylabel('y axis')
        plt.xticks(range(0, self.number_cells), range(0, self.number_cells))
        plt.yticks(range(0, self.number_cells), reversed(range(0, self.number_cells)))

    def annotate(self):
        threshold = self.getGridMaxValue()/2
        for x in range(self.number_cells):
            for y in range(self.number_cells):
                value = self.grid[self.number_cells-y-1][x]
                if value > threshold:
                    color = 'whitesmoke'
                else:
                    color = 'dimgray'
                plt.text(x, y, str(value), horizontalalignment='center', verticalalignment='center', color=color)

    def show(self):
        # Show the plot.
        plt.show()

    def save(self, path):
        NOW = datetime.now()
        DT_STRING = NOW.strftime("%d-%m-%Y_%H-%M-%S")
        fileName = path + "/" + DT_STRING + ".png"
        try:
            os.makedirs(os.path.dirname(fileName))
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise
        plt.savefig(fileName, bbox_inches='tight')
        print("File saved as: " + fileName)

    def randomPath(self, start_x, start_y, linear_speed, angular_speed, curliness):
        direction = random()*2*math.pi
        x = start_x
        y = start_y
        point_list = []
        point_list.append((x, y))
        curving = 1
        while x < (self.x_min+self.area_width) and y < (self.y_min+self.area_width) and x >= self.x_min and y >= self.y_min:
            x += math.cos(direction)*linear_speed*(random()/2 + 0.5)
            y += math.sin(direction)*linear_speed*(random()/2 + 0.5)
            if random() <= (1-curliness):
                curving *= -1
            direction += angular_speed*curving*(random()/2 + 0.5)
            point_list.append((x, y))
        return point_list[:-1]

    @classmethod
    def euclid_distance(cls, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    @classmethod
    def cleanPointList(cls, point_list, threshold):
        new_list = []
        new_list.append(point_list[0])
        last_point = point_list[0]
        for point in point_list:
            if cls.euclid_distance(point, last_point) > threshold:
                new_list.append(point)
                last_point = point
        return new_list

    @staticmethod
    def truncatePointList(point_list, decimal_places):
        new_list = []
        factor = 10.0 ** decimal_places
        for point in point_list:
            new_list.append(( math.trunc(point[0]*factor)/factor , math.trunc(point[1]*factor)/factor ))
        return new_list


if __name__ == "__main__":

    try:

        X_MIN = -10
        Y_MIN = -10
        WIDTH = 20
        N_CELLS = 20

        ROS = False
        SAVE = False
        SHOW = False
        if(len(sys.argv) > 1):
            for argument in sys.argv:
                if argument == "-ros":
                    ROS = True
                if argument == "-save":
                    SAVE = True
                if argument == "-show":
                    SHOW = True

        if ROS:
            rospy.init_node("demo_heatmap_node", anonymous=True)

            PKG_NAME = "drone_middleware"
            rospack = rospkg.RosPack()
            PKG_PATH = rospack.get_path(PKG_NAME)
            FILE_PATH = PKG_PATH + "/heatmaps"

            dm = DM_Handler()

            dm.waitForServices()
            dm.waitForStepServer()

            heatmap_data = dm.getHeatMap()
            #print(heatmap_data)
            X_MIN = heatmap_data.x_min
            Y_MIN = heatmap_data.y_min
            WIDTH = heatmap_data.width
            N_CELLS = heatmap_data.n_cells

        heatmap = MPL_Heatmap(N_CELLS, WIDTH, X_MIN, Y_MIN, interpolation='bilinear')
        
        if not ROS:
            heatmap.clearGrid()
            my_path = heatmap.randomPath(0, 0, 0.5, 0.25, 0.94)
            heatmap.incrementCellFromList(my_path)

            print(len(my_path))
            my_path2 = MPL_Heatmap.truncatePointList(my_path, 2)
            my_path2 = MPL_Heatmap.cleanPointList(my_path2, 0.5)
            print(len(my_path2))

            print(my_path2)
        else:
            heatmap.gridFromArray(heatmap_data.heatMap)

        heatmap.generate()
        heatmap.annotate()

        if ROS and SAVE:
            heatmap.save(FILE_PATH)

        if SHOW:
            heatmap.show()

    except rospy.exceptions.ROSInterruptException as e1:
        print(e1)
    
    except KeyboardInterrupt as e2:
        print("Interrupted by the user.")