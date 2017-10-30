#! /usr/bin/python
""" 
Visualising data from JSBSim. The script reads the files indicated by UAV_FILE and UGV_FILE and lets you plot the data. The plotting works at runtime.  
Linnea Persson 2017
laperss@kth.se
"""
from __future__ import print_function
import sys
import subprocess
import time
import os
import pandas as pd
from PyQt4 import QtCore, QtGui
import pyqtgraph.exporters
import pyqtgraph as pg

TITLE = 'Data Visualization'
UPDATE_FREQ = 10
PATH = os.path.dirname(os.path.abspath(__file__))
UAV_FILE = './logs/uav.csv'
UGV_FILE = './logs/ugv.csv'
COLORS = ['y', 'm', 'c', 'r', 'b', 'g', 'w','y', 'm', 'c', 'r', 'b', 'g', 'w']

# MAIN WINDOW
#---------------------------------------------------------------------------------
class VisualizationGraph(QtGui.QWidget):
    """ Base GUI class for visualising JSBSim/FlightGear landing. """

    color_it = 0
    styles = [QtCore.Qt.SolidLine, QtCore.Qt.DashLine, QtCore.Qt.DashDotLine]
    time = 0
    sliding_window_time = 30
    m = 2
    n = 2
    ugv_modtime = 0
    uav_modtime = 0
    uav_lines = []
    ugv_lines = []

    def __init__(self):
        QtGui.QWidget.__init__(self)
        self.setGeometry(0, 53, 1300, 700)
        self.main_layout = QtGui.QGridLayout(self)
        self.create_graph()
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(UPDATE_FREQ)

    def create_graph(self):
        """ Create the graph canvas """
        self.graphics_layout = pg.GraphicsLayoutWidget()

        self.graph = pg.PlotItem()
        self.graph.showGrid(True, True)
        self.graph.setLabel('bottom', 'Time [s]')
        self.graph.setTitle(TITLE)
        self.graph.setLimits(**{'xMin':0, 'minYRange':0.1})
        self.graph.addLegend(size=(20, 20), offset=(10, 10))
        self.graphics_layout.addItem(self.graph)

        menu = self.create_graph_selection_boxes()
        self.create_side_menu()
        self.main_layout.addWidget(self.graphics_layout, 0, 0, 4, 3)
        self.main_layout.addLayout(menu, 5, 0, 1, 3)


    def create_graph_selection_boxes(self):
        """ Create boxes for selecting data to show """
        if os.path.isfile(UAV_FILE):
            data = pd.read_csv(UAV_FILE)
            self.uav_headers = list(data)
            self.uav_file_exists = True

        if os.path.isfile(UGV_FILE):
            if self.ugv_modtime < os.stat(UGV_FILE)[8]:
                data = pd.read_csv(UGV_FILE)
                self.ugv_headers = list(data)
                self.ugv_file_exists = True

        drop_down_layout = QtGui.QGridLayout()

        self.scroll_box_uav = QtGui.QComboBox()
        self.scroll_box_ugv = QtGui.QComboBox()
        for item in self.ugv_headers[1:]:
            self.scroll_box_ugv.addItem('/ugv/'+item[12:])
        for item in self.uav_headers[1:]:
            self.scroll_box_uav.addItem('/uav/'+item[12:])

        add_uav_btn = QtGui.QPushButton("Add")
        add_uav_btn.clicked.connect(self.add_uav_plot)
        add_ugv_btn = QtGui.QPushButton("Add")
        add_ugv_btn.clicked.connect(self.add_ugv_plot)
        drop_down_layout.addWidget(self.scroll_box_uav, 0, 0, 1, 1)
        drop_down_layout.addWidget(self.scroll_box_ugv, 1, 0, 1, 1)
        drop_down_layout.addWidget(add_uav_btn, 0, 1, 1, 1)
        drop_down_layout.addWidget(add_ugv_btn, 1, 1, 1, 1)

        self.graph_menu_items = [self.scroll_box_uav,self.scroll_box_ugv,add_uav_btn,add_ugv_btn]
        return drop_down_layout

    def create_side_menu(self):
        """ Create the side menu """
        self.side_layout = QtGui.QGridLayout()
        self.side_layout.addWidget(QtGui.QLabel('Plotted Lines: '), 0, 0, 1, 1)
        self.main_layout.addLayout(self.side_layout, 0, 4, 1, 1)

    def update_side_menu(self):
        """ Update the side menu """
        for i in reversed(range(self.side_layout.count())): 
            self.side_layout.itemAt(i).widget().setParent(None)

        self.side_layout.addWidget(QtGui.QLabel('Plotted Lines: '), 0, 0, 1, 1)
        self.side_layout.addWidget(QtGui.QLabel('UAV'), 1, 0, 1, 1)
        for i in range(len(self.uav_lines)):
            line = self.uav_lines[i]
            button = QtGui.QToolButton()
            button.setText('Delete')
            button.setObjectName(line.name())
            button.released.connect(self.delete_line)
            self.side_layout.addWidget(QtGui.QLabel(line.name()), i + 2, 0, 1, 1)
            self.side_layout.addWidget(button, i + 2, 1, 1, 1)
        k = len(self.uav_lines) + 2
        self.side_layout.addWidget(QtGui.QLabel('UGV'),  k + 1, 0, 1, 1)
        for i in range(len(self.ugv_lines)):
            line = self.ugv_lines[i]
            button = QtGui.QToolButton()
            button.setText('Delete')
            button.setObjectName(line.name())
            button.released.connect(self.delete_line)
            self.side_layout.addWidget(QtGui.QLabel(line.name()), k + i + 2, 0, 1, 1)
            self.side_layout.addWidget(button, k + i + 2, 1, 1, 1)

    def delete_line(self):
        """ Delete one of the lines to plot """
        sending_button = self.sender()
        name = sending_button.objectName()
        print(sending_button.objectName())
        if name[1:4] == 'uav':
            for line in self.uav_lines:
                if line.name() == name:
                    self.uav_lines.remove(line)
                    self.graph.removeItem(line)
                    break
            self.uav_modtime = 0
        elif name[1:4] == 'ugv':
            for line in self.ugv_lines:
                if line.name() == name:
                    self.ugv_lines.remove(line)
                    self.graph.removeItem(line)
                    break
            self.ugv_modtime = 0
        self.graph.legend.removeItem(line.name())
        self.update_side_menu()

    def add_uav_plot(self):
        """ Add another variable to plot """
        c = COLORS[self.color_it]
        text = str(self.scroll_box_uav.currentText())
        if text not in [line.name() for line in self.uav_lines]: 
            line = (pg.PlotDataItem(x=[], y=[], name=text, pen=pg.mkPen(color=c, width=1.7)))
            line.dataName = '/fdm/jsbsim/'+text[5:]
            if ("-rad" in text or "gamma" in text or "theta" or "heading" in text):
                line.scale = 180/3.1415
            self.uav_lines.append(line)
            self.graph.addItem(line)
            self.uav_modtime = 0
        self.color_it += 1
        self.update_side_menu()

    def add_ugv_plot(self):
        """ Add another variable to plot """
        c = COLORS[self.color_it]
        text = str(self.scroll_box_ugv.currentText())
        if text not in [line.name() for line in self.ugv_lines]: 
            line = (pg.PlotDataItem(x=[], y=[], name=text,pen=pg.mkPen(color=c, width=1.7)))
            line.dataName = '/fdm/jsbsim/'+text[5:]
            if ("-rad" in text or "gamma" in text or "theta" or "heading" in text):
                line.scale = 180/3.1415
            self.ugv_lines.append(line)
            self.graph.addItem(line)
            self.ugv_modtime = 0
        self.color_it += 1
        self.update_side_menu()

    def update_plot(self):
        """ Calls the update_data() function if there has been
            any changes to the log files """
        if os.path.isfile(UAV_FILE):
            if self.uav_modtime < os.stat(UAV_FILE)[8]:
                self.update_data_file(UAV_FILE)
                self.uav_modtime = os.stat(UAV_FILE)[8]

        if os.path.isfile(UGV_FILE):
            if self.ugv_modtime < os.stat(UGV_FILE)[8]:
                self.update_data_file(UGV_FILE)
                self.ugv_modtime = os.stat(UGV_FILE)[8]

    def update_data_file(self, path):
        """ Read the updated data from a csv file. """
        data = pd.read_csv(path)
        idx_1 = len(data)
        idx_0 = max(0, len(data)-2500)
        data = data[idx_0:idx_1]

        if 'Rascal' in path:
            for line in self.uav_lines:
                xdata = data['Time'].ravel()
                try:
                    ydata = data[line.dataName].ravel()*line.scale
                except:
                    ydata = data[line.dataName].ravel()
                line.setData(xdata, ydata)
            self.uav_data = data
        elif 'followme' in path:
            for line in self.ugv_lines:
                xdata = data['Time'].ravel()
                try:
                    ydata = data[line.dataName].ravel()*line.scale
                except:
                    ydata = data[line.dataName].ravel()
                line.setData(xdata, ydata)
            self.ugv_data = data


app = QtGui.QApplication(sys.argv)
gui = VisualizationGraph()
gui.show()
app.exec_()

