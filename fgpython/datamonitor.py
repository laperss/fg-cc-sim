#! /usr/bin/python
""" 
Visualising data from JSBSim. 

Author: Linnea Persson, laperss@kth.se 

The script reads the files indicated by UAV_FILE and UGV_FILE and lets you plot the data. 
The plotting works at runtime.  

"""
from __future__ import print_function
import sys
import os
import pickle
from PyQt4 import QtCore, QtGui
import pyqtgraph.exporters
import pyqtgraph as pg
from PyQt4 import QtGui, QtCore
import pandas as pd

TITLE = 'Data Visualization'
UPDATE_FREQ = 10
FEET2M = 0.3048

PATH = os.path.dirname(os.path.abspath(__file__))
UAV_FILE = './logs/Rascal.csv'
USV_FILE = './logs/USV.csv'
UGV_FILE = './logs/ground-vehilce.csv'

# MAIN PROGRAM
#---------------------------------------------------------------------------------
class VisualizationGraph(QtGui.QWidget):
    """ Base GUI class for visualising JSBSim/FlightGear landing. """

    colors = ['y', 'm', 'c', 'r', 'b', 'g', 'w','y', 'm', 'c', 'r', 'b', 'g', 'w']
    color_it = 0
    styles = [QtCore.Qt.SolidLine, QtCore.Qt.DashLine, QtCore.Qt.DashDotLine]
    lines_mpc = dict()

    def __init__(self, vehicles):
        QtGui.QWidget.__init__(self)
        self.vehicles = []
        for v in vehicles: 
            vehicle = {'lines':[], 'prop':[], 'modtime':0}
            vehicle['name'] = v[0]
            vehicle['file'] = v[1]
            self.vehicles.append(vehicle)
        self.setGeometry(0, 53, 1300, 700)
        self.main_layout = QtGui.QGridLayout(self)
        self.create_graph()
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(UPDATE_FREQ)

        self.loadData()
    def closeEvent(self, event):
        """ Close the window and end process. """
        vehicles = dict()
        for vehicle in self.vehicles:
            lines = [line.name() for line in vehicle['lines']]
            vehicles[vehicle['name']] = lines
        pickle.dump(vehicles, open("save.p", "wb" ))
        event.accept() 

    def loadData(self):
        """ Load saved data from pickle """
        try:
            lines = pickle.load(open("save.p", "rb" ))
            for vehicle in self.vehicles:
                for line in lines[vehicle['name']]:
                    try:
                        self.add_plot(vehicle, line)
                    except:
                        pass
        except EOFError:
            pass

    def create_graph(self):
        """ Create the graph canvas """
        self.graphics_layout = pg.GraphicsLayoutWidget()

        self.graph = pg.PlotItem()
        self.graph.showGrid(True, True)
        self.graph.setLabel('bottom', 'Time [s]')
        self.graph.setTitle('FlightGear simulation data')
        self.graph.setLimits(**{'xMin':0, 'minYRange':0.1})
        self.graph.addLegend(size=(20, 20), offset=(10, 10))
        self.graphics_layout.addItem(self.graph)
        menu = self.create_graph_selection_boxes()
        self.create_side_menu()
        self.main_layout.addWidget(self.graphics_layout, 0, 0, 4, 3)
        self.main_layout.addLayout(menu, 5, 0, 1, 2)


    def create_graph_selection_boxes(self):
        """ Create boxes for selecting data to show """
        drop_down_layout = QtGui.QGridLayout()

        for vehicle in self.vehicles:
            vehicle['scroller'] = QtGui.QComboBox()

        self.update_headers()
        i = 1
        for vehicle in self.vehicles:
            drop_down_layout.addWidget(vehicle['scroller'], i, 0, 1, 3)
            add_btn = QtGui.QPushButton("Add") 
            add_btn.clicked.connect(lambda arg, v=vehicle: self.add_plot_callback(v))
            drop_down_layout.addWidget(add_btn, i, 3, 1, 1)
            i += 1

        update_btn = QtGui.QPushButton("Update")
        update_btn.clicked.connect(self.update_headers)
        drop_down_layout.addWidget(update_btn, 1, 4, 1, 1)
        update_btn = QtGui.QPushButton("Clear")
        update_btn.clicked.connect(self.clear_headers)
        drop_down_layout.addWidget(update_btn, 2, 4, 1, 1)
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

        k = 0
        for vehicle in self.vehicles:
            text = QtGui.QLabel('%s\n' %vehicle['name'])
            text.setAlignment(QtCore.Qt.AlignCenter)
            text.setStyleSheet('font-weight: 500;text-decoration:underline;')
            self.side_layout.addWidget(text, 1 + k, 0, 1, 1)
            i = 1
            for line in vehicle['lines']:
                button = QtGui.QToolButton()
                button.setText('Delete')
                button.setObjectName(line.name())
                button.released.connect(self.delete_line)
                self.side_layout.addWidget(QtGui.QLabel(line.name()), k + i + 2, 0, 1, 1)
                self.side_layout.addWidget(button, k + i + 2, 1, 1, 1)
                i += 1
            k = k + i + 2

    def update_headers(self):
        """ Get the headers of the vehicle CSV data files """
        for vehicle in self.vehicles:
            if os.path.isfile(vehicle['file']):
                data = pd.read_csv(vehicle['file'])
                vehicle['headers'] = list(data)

        for vehicle in self.vehicles:
            vehicle['scroller'].clear()
            for item in vehicle['headers'][1:]:
                vehicle['scroller'].addItem('/'+ vehicle['name']+'/'+item[12:])

    def clear_headers(self):
        """ Clear the plots of all headers """
        for vehicle in self.vehicles:
            for line in vehicle['lines']:
                self.graph.removeItem(line)
                vehicle['modtime'] = 0
                self.graph.legend.removeItem(line.name())
            vehicle['lines'] = []
        self.update_side_menu()

    def delete_line(self):
        """ Delete one of the lines to plot """
        name = self.sender().objectName()

        for vehicle in self.vehicles:
            if name in [v.name() for v in vehicle['lines']]:
                line, = [l for l in vehicle['lines'] if l.name() == name]
                vehicle['lines'].remove(line)
                self.graph.removeItem(line)
                vehicle['modtime'] = 0
                break

        self.graph.legend.removeItem(line.name())
        self.update_side_menu()

    def add_plot_callback(self, vehicle):
        """ Add another variable to plot """
        c = self.colors[self.color_it]
        text = str(vehicle['scroller'].currentText())
        if text not in [line.name() for line in vehicle['lines']]:
            vehicle['prop'].append(text)
            line = (pg.PlotDataItem(x=[], y=[], name=text,pen=pg.mkPen(color=c, width=1.7)))
            line.dataName = '/fdm/jsbsim/'+text[5:]
            vehicle['lines'].append(line)
            self.graph.addItem(line)
            vehicle['modtime'] = 0
        self.color_it += 1
        self.update_side_menu()

    def add_plot(self, vehicle, linename):
        """ Add another variable to plot """
        c = self.colors[self.color_it]
        if linename not in [line.name() for line in vehicle['lines']]:
            vehicle['prop'].append(linename)
            line = (pg.PlotDataItem(x=[], y=[], name=linename,pen=pg.mkPen(color=c, width=1.7)))
            line.dataName = '/fdm/jsbsim/'+linename[5:]
            vehicle['lines'].append(line)
            self.graph.addItem(line)
            vehicle['modtime'] = 0
        self.color_it += 1
        self.update_side_menu()

    def plot(self, x, y, prop, c='r'):
        if prop not in self.lines_mpc.keys():
            line = (pg.PlotDataItem(x=x, y=y, name=prop, dataName=prop,pen=pg.mkPen(color=c, width=1.7)))
            self.graph.addItem(line)
            self.lines_mpc[prop] = line
            line.setData(x, y)
        else:
            self.lines_mpc[prop].setData(x, y)

    def update_plot(self):
        """ Calls the update_data() function if there has been
            any changes to the log files """
        for vehicle in self.vehicles:
            if os.path.isfile(vehicle['file']):
                if vehicle['modtime'] < os.stat(vehicle['file'])[8]:
                    self.update_data_file(vehicle['file'])
                    vehicle['modtime'] = os.stat(vehicle['file'])[8]

    def update_data_file(self, path):
        """ Read the updated data from a csv file. """
        data = pd.read_csv(path)
        idx_1 = len(data)
        idx_0 = max(0, len(data)-2500)
        data = data[idx_0:idx_1]

        for vehicle in self.vehicles:
            if path == vehicle['file']:
                for line in vehicle['lines']:
                    xdata = data['Time'].ravel()
                    try:
                        ydata = data[line.dataName].ravel()
                    except:
                        ydata = data[line.dataName].ravel()
                    line.setData(xdata, ydata)


app = QtGui.QApplication(sys.argv)
gui = VisualizationGraph([['UAV', UAV_FILE], ['USV', USV_FILE]])
gui.show()
app.exec_()

