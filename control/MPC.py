#! /usr/bin/python
""" Optimization and controller related functions 

Author: Linnea Persson, laperss@kth.se

Usage: This is a specific controller for the Rascal and followme vehicles. 
Load it into the main file as a part of the control thread:
        uav = MPC.Vehicle()
        ugv = MPC.Vehicle()
Add constraints: 
        mpc_lat = MPC.Controller([uav.x, uav.y, uav.v, uav.a, uav.chi,
                                  ugv.x, ugv.y, ugv.v, ugv.a, ugv.chi],
                             [uav.a_des, uav.chi_des,
                              ugv.a_des, ugv.chi_des],
                             MPC.A_lat, MPC.B_lat, T, self.ds)

        mpc_alt = MPC.Controller([uav.h, uav.gamma],
                                 [uav.gamma_des],
                                 MPC.A_lon, MPC.B_lon, T, self.ds)
The objective functions must be initiated: 
        MPC.add_align_constraints(mpc)
        MPC.add_alt_constraints(mpc_alt) 

"""
from __future__ import print_function
import math
import time
import cvxpy
import numpy as np
import matplotlib.pyplot as plt

                   #  x    y    v     a     psi
A_lat = np.matrix([[1.0, 0.0, 0.18, 0.01448, 0.0,      0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0, 0.0,      3.31,     0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.0, 0.0, 1.0, 0.1521,   0.0,      0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.0, 0.0, 0.0, 0.7065,   0.0,      0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.0, 0.0, 0.0, 0.0,    0.843,      0.0, 0.0, 0.0, 0.0, 0.0],
                   #                           x    y    v         a         psi
                   [0.0, 0.0, 0.0, 0.0, 0.0,   1.0, 0.0, 0.18,     0.003146, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0,   0.0, 1.0, 0.0,      0.0,      3.127],
                   [0.0, 0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 1.0,      0.0262,   0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0,   0.0, 0.0, -0.0006,  0.1156,   0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0,      0.0,      0.749]])


B_lat = np.matrix([[0.0016,  0.0, 0.0, 0.0,      0.0,     0.0],
                   [0.0,     0.0, 0.0, 0.0,      0.01517, 0.2789],
                   [0.0257,  0.0, 0.0, 0.0,      0.0,     0.0],
                   [0.2701,  0.0, 0.0, 0.0,      0.0,     0.0],
                   [0.0,     0.0, 0.0, 0.0,      0.03768, 0.1212], 
#
                   [0.0,     0.0, 0.0129, 0.0,    0.0,    0.0],
                   [0.0,     0.0, 0.0,    0.488,  0.0,    0.0],
                   [0.0,     0.0, 0.1518, 0.0,    0.0,    0.0],
                   [0.0,     0.0, 0.8517, 0.0,    0.0,    0.0],
                   [0.0,     0.0, 0.0,    0.2532, 0.0,    0.0]])

A_lon = np.matrix([[ 1., 3.925],
                   [ 0., 0.746]])

B_lon = np.matrix([[-0.2788],
                   [0.2508]])


def add_align_constraints(ctrl):
    ctrl.past_input = cvxpy.Variable(4, ctrl.T+4, "past_input")

    deltax = cvxpy.sum_squares(ctrl.x[0, 2:] - ctrl.x[5, 2:])
    deltay = cvxpy.sum_squares(ctrl.x[1, 10:] - ctrl.x[6, 10:])

    # Distance -> 0
    ctrl.objective.append(1.0*deltax)
    ctrl.objective.append(1.0*deltay)

    ctrl.objective.append(cvxpy.sum_squares(ctrl.x[3, 2:]))   # a
    ctrl.objective.append(cvxpy.sum_squares(ctrl.x[8, 2:]))   # a

    for t in range(ctrl.T):
        ctrl.objective.append(2*cvxpy.sum_squares(ctrl.u[:, t]))

    for t in range(2,ctrl.T):
        # UAV velocity -> 20 
        ctrl.objective.append(cvxpy.sum_squares(ctrl.x[2, t] - 20))
        ctrl.objective.append(cvxpy.sum_squares(ctrl.x[7, t] - 20))
        # Rate of change, acceleration
        ctrl.objective.append(1*cvxpy.sum_squares(ctrl.u[2, t] - ctrl.u[2, t-1]))
        ctrl.objective.append(3*cvxpy.sum_squares(ctrl.u[0, t] - ctrl.u[0, t-1]))

    ### END CONSTRAINTS ###
    # Distance
    ctrl.constraints += [ctrl.x[0, -1] - ctrl.x[5, -1] <= 1.0]    # deltax
    ctrl.constraints += [ctrl.x[0, -1] - ctrl.x[5, -1] >= -1.0] 
    ctrl.constraints += [ctrl.x[1, -1] - ctrl.x[6, -1] <= 1.0]    # deltay
    ctrl.constraints += [ctrl.x[1, -1] - ctrl.x[6, -1] >= -1.0]
    # Velocity
    ctrl.constraints += [ctrl.x[2, -1] - ctrl.x[7, -1] == 0] 
    # Acceleration
    ctrl.constraints += [ctrl.x[3, -1] - ctrl.x[8, -1] == 0] 

    ### Dynamic constriants ###
    # Include past inputs: 0 = t-4, 1 = t-3, 2 = t-2, 3 = t-1, 4=t
    ctrl.constraints += [ctrl.past_input[:,0:4] == ctrl.u_delay]
    ctrl.constraints += [ctrl.past_input[:,4:] == ctrl.u[:,:]]

    ctrl.constraints += [ctrl.x[:, 1:ctrl.T+1] == ctrl.A*ctrl.x[:, 0:ctrl.T]
                         + ctrl.B[:,0]*ctrl.u[0,0:ctrl.T]           #a_uav(t)
                         + ctrl.B[:,2]*ctrl.u[2,0:ctrl.T]           #a_ugv(t)
                         + ctrl.B[:,3]*ctrl.u[3,0:ctrl.T]           #psi_ugv(t)
                         + ctrl.B[:,4]*ctrl.past_input[1, 2:ctrl.T+2] #psi_uav(t - 2) 
                         + ctrl.B[:,5]*ctrl.past_input[1, 1:ctrl.T+1] #psi_uav(t - 3) 
]

    return ctrl

d_safe = 2.3
d_land = 1
h_safe = 5
d_safe_s = d_safe - 1
d_land_s = d_land - 0.5
h_safe_s = h_safe + 0.6

def add_alt_constraints(ctrl):
    """ Add the altitide constraints for the longitudinal MPC """
    ctrl.b1 = cvxpy.Parameter(ctrl.T)
    ctrl.b2 = cvxpy.Parameter(ctrl.T)
    ctrl.distance = cvxpy.Parameter(ctrl.T, sign="positive")

    ctrl.s1 = cvxpy.Variable(3, ctrl.T-1)

    ctrl.objective.append(1*cvxpy.sum_squares(ctrl.x[0, ctrl.T])    # End altitude
                          + 1*cvxpy.sum_squares(ctrl.x[1,:])      # Angle size
                          + 500*cvxpy.sum_squares(ctrl.s1)          # Slack variable
                          )
    ctrl.objective.append(cvxpy.sum_entries(30.*ctrl.b1*ctrl.x[0,:]))

    # Positive altitude
    ctrl.constraints +=  [ctrl.x[0,1:]  >= -2]

    # Landing constraints - Stay within safe set
    ctrl.constraints += [ctrl.x[0,1:] >= h_safe - 1000*ctrl.b1.T]
    ctrl.constraints += [(d_safe - d_land)*ctrl.x[0,1:] + h_safe*ctrl.distance.T >= -h_safe*d_land - 1000.*ctrl.b2.T]
    ctrl.constraints += [(d_safe - d_land)*ctrl.x[0,1:] - h_safe*ctrl.distance.T >= -h_safe*d_land - 1000.*ctrl.b2.T]

    ctrl.constraints +=  [ctrl.x[0,2:] >= (h_safe_s - ctrl.s1[0,:]) -1000.*ctrl.b1[1:].T] 
    ctrl.constraints += [(d_safe_s - d_land_s)*(ctrl.x[0,2:]) + h_safe_s*ctrl.distance[1:].T >= - (h_safe_s)*(d_land_s) - ctrl.s1[1,:] - 5000.*ctrl.b2[1:].T]
    ctrl.constraints += [(d_safe_s - d_land_s)*(ctrl.x[0,2:]) - h_safe_s*ctrl.distance[1:].T >= - (h_safe_s)*(d_land_s) - ctrl.s1[2,:] - 5000.*ctrl.b2[1:].T]

    ctrl.constraints += [ctrl.s1[0,:]   <= 0.6, 
                         ctrl.s1[1,:] <= 10,
                         ctrl.s1[2,:] <= 10]

    # Vertical touchdown velocity
    gamma_td = -2
    gamma_lim = -5
    h_flare = 5
    ctrl.constraints += [ctrl.x[1,1:ctrl.T+1] >= (gamma_lim-gamma_td)/h_flare*(ctrl.x[0,1:ctrl.T+1]) + gamma_td]

    # Dynamic constraints
    ctrl.constraints += [ctrl.x[:, 1:ctrl.T+1] == ctrl.A*ctrl.x[:, 0:ctrl.T]
                         + ctrl.B*ctrl.u[:, 0:ctrl.T]]


class State(object):
    """ Representing a state in the MPC optimization """
    def __init__(self, initial=0, min_=-10000, max_=10000):
        self.min_ = min_
        self.max_ = max_
        self.initial = initial

class Vehicle(object):
    def __init__(self):
        self.x = State(0, -10000, 10000)
        self.y = State(0, -10000, 10000)
        self.h = State(20, -1, 150)
        self.v = State(20, 18, 27.5)
        self.a = State(0, -1.5, 1.5)
        self.chi = State(0, math.radians(-10), math.radians(10))
        self.gamma = State(0, math.radians(-7), math.radians(15))
        self.chi_des = State(0, math.radians(-10), math.radians(10))
        self.gamma_des = State(0, math.radians(-5), math.radians(10))
        self.a_des = State(0, -1.5, 1.5)

class Controller(object):
    """ A class for executing the model predictive control"""
    def __init__(self, states, inputs, A, B, T, ds):
        self.T = T
        self.ds = ds
        self.states = states
        self.inputs = inputs
        self.n = len(states)
        self.m = len(inputs)
        self.A = A
        self.B = B
        # states/inputs
        self.x = cvxpy.Variable(self.n, self.T + 1, "x")
        self.u = cvxpy.Variable(self.m, self.T, "u")
        # slack variables
        self.s0 = cvxpy.Variable(self.n, self.T, "s")
        # initial conditions
        self.x0 = cvxpy.Parameter(self.n,1, "x0")
        self.u0 = cvxpy.Parameter(self.m,1, "u0")
        self.u_delay = cvxpy.Parameter(self.m,4, "u_delay")

        self.initial = [state.initial for state in states]
        self.constraints = self.init_constraints()
        self.objective = self.init_objective()

    def init_constraints(self):
        """ Add limits and state dynamics constraints to the constraint vector"""
        # Initial conditions
        constraints = [self.x[:,0] == self.x0]
        constraints += [self.u[:,0] == self.u0]
        # State and input constraints
        for i in range(self.m):
            constraints += [self.u[i, :] <= self.inputs[i].max_]
            constraints += [self.u[i, :] >= self.inputs[i].min_]
        for i in range(self.n):
            constraints += [self.x[i, 2:] <= self.states[i].max_ + self.s0[i, 1:]]
            constraints += [self.x[i, 2:] >= self.states[i].min_ - self.s0[i, 1:]]
            # slack variable constraint
            constraints += [self.s0[i, :] > 0]
            constraints += [self.s0[i, :] <= self.states[i].max_/10]

        return constraints

    def init_objective(self):
        """ Initialize the objective function """
        objective = [500*cvxpy.sum_squares(self.s0)]
        return objective

    def solve(self, initial_state, initial_input = [0,0,0,0], distance = None):
        """ Add initial conditions to optimization problem """
        self.x0.value = initial_state
        self.u0.value = initial_input
        objective = sum(self.objective)
        problem = cvxpy.Problem(cvxpy.Minimize(objective), self.constraints)
        result = problem.solve(verbose=False)
        return result

