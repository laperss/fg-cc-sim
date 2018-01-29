# FlightGear cooperative control simulations
This repository shows how to implement an experimental setup environment for testing cooperative maneuvers in FlightGear. 

<p align="center">
  <img width="430" height="300" src="https://user-images.githubusercontent.com/4593893/35376014-867d72a4-01a9-11e8-8340-c74e458e684c.png">
  <img width="430" height="300" src="https://people.kth.se/~laperss/assets/images/fg_landing.png">
</p>


## Requirements
### Hardware
I ran the simulations on a Intel Core i7 CPU 3.40 GHz x 8 stationary computer running Ubuntu 16.04.
### Software
This is the software required to run the simulations. In bracket is the version it has been tested with.
* [FlightGear](http://www.flightgear.org/download/) [v2017.2.1]
* [JSBSim](https://sourceforge.net/projects/jsbsim/) [v1.0]
* Python 3 [3.6]

Some optional packages
* [PyQT](https://wiki.python.org/moin/PyQt) and [PyQTgraph](http://www.pyqtgraph.org/) for plotting and GUI

#### FlightGear Multiplay
In order for the vehicle simulations to be displayed together, FlightGear multiplay mode is used. There is a delay in the transmission of data between the instances, causing lag in the visualization. To better this, enable multiplayer lag correction in FlighGear: Multiplayer -> Lag Settings. 
