# FlightGear cooperative control simulations
This repository shows how to implement an experimental setup environment for testing cooperative maneuvers in FlightGear. 

<p align="center">
  <img width="430" height="300" src="https://user-images.githubusercontent.com/4593893/35376014-867d72a4-01a9-11e8-8340-c74e458e684c.png">
  <img width="430" height="300" src="https://people.kth.se/~laperss/assets/images/fg_landing.png">
</p>

# Citing
Main paper han be found [here](https://ieeexplore.ieee.org/abstract/document/8550247). 

**Text citation**
```
L. Persson and B. Wahlberg, "Verification of Cooperative Maneuvers in FlightGear using MPC and Backwards Reachable Sets," 2018 European Control Conference (ECC), Limassol, 2018, pp. 1411-1416.
```

**BibTeX citation**
```
@inproceedings{persson2018verification,
  title={Verification of cooperative maneuvers in flightgear using mpc and backwards reachable sets},
  author={Persson, Linnea and Wahlberg, Bo},
  booktitle={2018 European Control Conference (ECC)},
  pages={1411--1416},
  year={2018},
  organization={IEEE}
}
```

## Before you start
* Update the path to JSBSim in `./flightgear/run_flightgear.sh`
* Copy the protocols to the Flightgear/Protocols folder

## Run the simulations
* An example simulation is run using the script `start_uav_ugv_sim.py`. Note that Python 3 has to be used. This should open a control GUI, from where FlightGear can be started with the `Start` button. 

## Dataplotter
The script ```fgpython/datamonitor.py``` is used to show the live data being written in the files found in the "logs" folder. 

<p align="center">
  <img width="500" height="300" src="https://user-images.githubusercontent.com/4593893/35514445-7a4a3cfa-0506-11e8-9f02-c4f4b5aa938a.png">
</p>


## Requirements
### Hardware
The simulations were run on a Intel Core i7 CPU 3.40 GHz x 8 stationary computer running Ubuntu 16.04.
### Software
This is the software required to run the simulations. In bracket is the version it has been tested with.
* [FlightGear](http://www.flightgear.org/download/) [v2017.2.1]
* [JSBSim](https://sourceforge.net/projects/jsbsim/) [v1.0]
* Python 3 [3.6]

Some optional packages
* [PyQT](https://wiki.python.org/moin/PyQt) and [PyQTgraph](http://www.pyqtgraph.org/) for plotting and GUI

#### FlightGear Multiplay
In order for the vehicle simulations to be displayed together, FlightGear multiplay mode is used. There is a delay in the transmission of data between the instances, causing lag in the visualization. To better this, enable multiplayer lag correction in FlighGear: Multiplayer -> Lag Settings. 
