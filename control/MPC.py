#! /usr/bin/python
""" Optimization and controller related functions 

Author: Linnea Persson, laperss@kth.se

This is a specific controller for the drone and ground rendezvous.

Usage:  
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
                   [0.0, 0.0, 0.0, 0.0, 0.0,   1.0, 0.0, 0.18,     0.009137, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0,   0.0, 1.0, 0.0,      0.0,      3.127],
                   [0.0, 0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 1.0,      0.08229,   0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.000126,  0.2342,   0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0,      0.0,      0.749]])


B_lat = np.matrix([[0.0016,  0.0, 0.0, 0.0,      0.0,     0.0],
                   [0.0,     0.0, 0.0, 0.0,      0.01517, 0.2789],
                   [0.0257,  0.0, 0.0, 0.0,      0.0,     0.0],
                   [0.2701,  0.0, 0.0, 0.0,      0.0,     0.0],
                   [0.0,     0.0, 0.0, 0.0,      0.03768, 0.1212], 
#
                   [0.0,     0.0, 0.0077, 0.0,    0.0,    0.0],
                   [0.0,     0.0, 0.0,    0.488,  0.0,    0.0],
                   [0.0,     0.0, 0.1024, 0.0,    0.0,    0.0],
                   [0.0,     0.0, 0.6856, 0.0,    0.0,    0.0],
                   [0.0,     0.0, 0.0,    0.2532, 0.0,    0.0]])

A_lon = np.matrix([[ 1., 3.925],
                   [ 0., 0.746]])

B_lon = np.matrix([[-0.2788],
                   [0.2508]])

# STATE CONSTRAINTS
constraints_uav = [(-10000, 10000),  # x
                   (-10000, 10000),  # y
                   (-1, 150),        # h
                   (18, 27.5),       # v
                   (-0.5, 1.0),      # a
                   (math.radians(-10), math.radians(10)), # heading
                   (math.radians(-5), math.radians(15))] # flight path


constraints_ugv = [(-10000, 10000), 
                   (-10000, 10000),
                   (0, 27.5),
                   (-1.0, 1.0),
                   (math.radians(-10), math.radians(10))]

# INPUT CONSTRAINTS
constraints_uav_input = [(-0.5, 1.0),      # a
                         (math.radians(-10), math.radians(10)), # heading
                         (math.radians(-7), math.radians(15))] # flight path
                    
constraints_ugv_input = [(-1.0, 1.0),      # a
                         (math.radians(-10), math.radians(10))] # heading


# Controller constraints 
state_constraints_lat = constraints_uav[0:2] + constraints_uav[3:6] + constraints_ugv[:]
state_constraints_lon = [constraints_uav[2], constraints_uav[6]]

input_constraints_lat = constraints_uav_input[0:2] + constraints_ugv[:]
input_constraints_lon = [constraints_uav_input[2]]


def add_align_constraints(ctrl):
    """ Add the alignment constraints for the longitudinal MPC """
    ctrl.u_delay = cvxpy.Parameter(4, 4, "u_delay")
    ctrl.past_input = cvxpy.Variable(4, ctrl.T + 4, "past_input")

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
                         + ctrl.B[:,0]*ctrl.u[0,0:ctrl.T]              #a_uav(t)
                         + ctrl.B[:,2]*ctrl.u[2,0:ctrl.T]              #a_ugv(t)
                         + ctrl.B[:,3]*ctrl.u[3,0:ctrl.T]              #psi_ugv(t)
                         + ctrl.B[:,4]*ctrl.past_input[1, 2:ctrl.T+2]  #psi_uav(t - 2)
                         + ctrl.B[:,5]*ctrl.past_input[1, 1:ctrl.T+1]] #psi_uav(t - 3)
    return ctrl

d_safe = 2.5
d_land = 1.2
h_safe = 5
d_safe_s = d_safe - 1
d_land_s = d_land - 0.5
h_safe_s = h_safe + 0.6

def add_alt_constraints(ctrl):
    """ Add the altitide constraints for the vertical MPC """
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
    gamma_td = math.radians(-2)
    gamma_lim = math.radians(-5)
    h_flare = 5
    ctrl.constraints += [ctrl.x[1,1:ctrl.T+1] >= (gamma_lim-gamma_td)/h_flare*(ctrl.x[0,1:ctrl.T+1]) + gamma_td]

    # Dynamic constraints
    ctrl.constraints += [ctrl.x[:, 1:ctrl.T+1] == ctrl.A*ctrl.x[:, 0:ctrl.T]
                         + ctrl.B*ctrl.u[:, 0:ctrl.T]]



class Controller(object):
    """ A class for executing the model predictive control"""
    def __init__(self, states, inputs, A, B, T, ds):
        self.T = T
        self.ds = ds
        self.states = states
        self.inputs = inputs
        self.n = A.shape[1]
        self.m = (len(inputs)+1)/2
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

        self.constraints = self.init_constraints()
        self.objective = self.init_objective()

    def init_constraints(self):
        """ Add limits and state dynamics constraints to the constraint vector"""
        # Initial conditions
        constraints = [self.x[:,0] == self.x0]
        constraints += [self.u[:,0] == self.u0]
        # State and input constraints
        for i in range(self.m):
            constraints += [self.u[i, :] <= self.inputs[i][1]]
            constraints += [self.u[i, :] >= self.inputs[i][0]]
        for i in range(self.n):
            constraints += [self.x[i, 2:] <= self.states[i][1] + self.s0[i, 1:]]
            constraints += [self.x[i, 2:] >= self.states[i][0] - self.s0[i, 1:]]
            # slack variable constraint
            constraints += [self.s0[i, :] > 0]
            constraints += [self.s0[i, :] <= self.states[i][1]/10]
        return constraints

    def init_objective(self):
        """ Initialize the objective function """
        objective = [500*cvxpy.sum_squares(self.s0)]
        return objective

    def solve(self, initial_state, initial_input = [0,0,0,0], distance = None):
        """ Add the initial conditions to the optimization problem """
        self.x0.value = initial_state
        self.u0.value = initial_input
        objective = sum(self.objective)
        problem = cvxpy.Problem(cvxpy.Minimize(objective), self.constraints)
        result = problem.solve(verbose=False)
        return result


def plot_altitude_result():
    h1 = alt_ctrl.x[0, :].value.A.flatten() # h
    h2 = alt_ctrl.x[1, :].value.A.flatten() # gamma

    v1 = alt_ctrl.u[0, :].value.A.flatten() # gamma des

    plt.figure(1)

    plt.plot(h1[1:], label='alt') 
    plt.plot(h2[1:]*180/3.14, label='gamma')
    plt.plot(v1[1:]*180/3.14, label='input')
    plt.legend(loc="upper right")

    plt.figure(2)
    plt.plot(alt_ctrl.distance.value, h1[0:-1])
    plt.plot([-5, -d_safe], [h_safe, h_safe], 'k')
    plt.plot([d_safe, 5], [h_safe, h_safe], 'k')
    plt.plot([d_safe, d_land], [h_safe, 0], 'k')
    plt.plot([-d_safe, -d_land], [h_safe, 0], 'k')

    plt.plot([-5, -d_safe_s], [h_safe_s, h_safe_s], 'r--')
    plt.plot([d_safe_s, 5], [h_safe_s, h_safe_s], 'r--')
    plt.plot([d_safe_s, d_land_s], [h_safe_s, 0], 'r--')
    plt.plot([-d_safe_s, -d_land_s], [h_safe_s, 0], 'r--')

    plt.ylim([-1, 10])
    plt.xlim([-5, 5])


    plt.show()


def plot_align_result(x, u):
    # UAV ----------------------------------------
    x1 = x[0, :].value.A.flatten() # x
    x2 = x[1, :].value.A.flatten() # y
    x3 = x[2, :].value.A.flatten() # v
    x4 = x[3, :].value.A.flatten() # a
    x5 = x[4, :].value.A.flatten() # chi
    # UGV ----------------------------------------
    x6 = x[5, :].value.A.flatten() # x
    x7 = x[6, :].value.A.flatten() # y
    x8 = x[7, :].value.A.flatten() # v
    x9 = x[8, :].value.A.flatten() # a
    x10 = x[9, :].value.A.flatten() # chi
    # UAV ----------------------------------------
    u1 = u[0, :].value.A.flatten() # a des
    u2 = u[1, :].value.A.flatten() # chi des
    u3 = u[2, :].value.A.flatten() # a des
    u4 = u[3, :].value.A.flatten() # chi des

    f = plt.figure()
    ax = f.add_subplot(221)
    plt.plot(u1, label=r'$a_{des, uav}$')
    plt.plot(u2*180/math.pi, label=r'$\chi_{des, uav}$')
    #plt.plot(v1*180/math.pi, label=r'$\gamma_{des, uav}$')
    plt.plot(u3, label=r'$a_{des, ugv}$')
    plt.plot(u4*180/math.pi, label=r'$\chi_{des, ugv}$')
    plt.ylabel(r"$input$", fontsize=16)
    plt.legend(loc="lower right")

    # Plot (u_t)_2.
    plt.subplot(2, 2, 2)
    plt.plot(x1-x6, label=r'$\Delta{x}$')
    plt.plot(x2-x7, label=r'$\Delta{y}$')
    #plt.plot(h1, label=r'$\Delta{h}$')
    plt.plot([0, 35], [1, 1], 'r--')
    plt.plot([0, 35], [-1, -1], 'r--')
    plt.ylabel(r"$distance$", fontsize=16)
    plt.ylim([-15, 25])
    plt.legend(loc="upper right")

    # Plot (x_t)_1.
    plt.subplot(2, 2, 3)

    #plt.plot(x1,label='x')
    #plt.plot(x2, label='y')
    plt.plot(x3, label='v')
    plt.plot(x4, label='a')
    plt.plot(u1, label=r'$a_{des}$')
    #plt.plot(x5, label=r'$\chi$')
    plt.ylabel("UAV", fontsize=16)
    plt.legend(loc="upper right")
    plt.subplot(2, 2, 4)

    #plt.plot(range(align_ctrl.T+1), x6,label='x')
    #plt.plot(range(align_ctrl.T+1), x7, label='y')
    plt.plot(x8, label='v')
    plt.plot(u3, label=r'$a_{des}$')
    plt.plot(x9, label='a')
    plt.plot((u3-x9[:-1]+30)*0.2, label='diff')

    #plt.plot(x10, label=r'$\chi$')

    plt.ylabel("UGV", fontsize=16)
    plt.xlabel(r"$t$", fontsize=16)
    plt.tight_layout()
    plt.legend(loc="upper right")


    f2 = plt.figure()
    plt.plot(u2*180/math.pi, label=r'$\chi_{des, uav}$')
    plt.plot(u4*180/math.pi, label=r'$\chi_{des, ugv}$')
    plt.plot(x5*180/math.pi, label=r'$\chi_{uav}$')
    plt.plot(x10*180/math.pi, label=r'$\chi_{ugv}$')
    #print(u2*180/math.pi)
    plt.legend(loc="upper right")


    plt.show()

if __name__ == "__main__":
    UAV = Vehicle()
    UGV = Vehicle()

    
    UAV.a.min_ = -0.5
    UAV.a.max_ = 1.0
    UAV.a_des.min_ = -0.5
    UAV.a_des.max_ = 1.0

    UGV.a_des.min_ = -1.3
    UGV.a_des.max_ = 1.3

    T = 36
    ds = 0.18
    ###################################################################    
    align_ctrl = Controller([UAV.x, UAV.y, UAV.v, UAV.a, UAV.chi,
                             UGV.x, UGV.y, UGV.v, UGV.a, UGV.chi],
                            [UAV.a_des, UAV.chi_des, UGV.a_des, UGV.chi_des],
                            A_lat, B_lat, T, ds)
    align_ctrl = add_align_constraints(align_ctrl)
    
    alt_ctrl = Controller([UAV.h, UAV.gamma],
                          [UAV.gamma_des],
                          A_lon, B_lon, T, ds)
    add_alt_constraints(alt_ctrl)


    align_ctrl.u_delay.value = [[0, -0.0010435454644829075, 0, 0], [0, -0.0006125127053132244, 0, 0], [0, 2.571279146357698e-05, 0, 0], [0, 0.0009590111970212249, 0, 0]]

    state_t = [4.00, 0.479, 24.39641, 0.6562344, 0.004001,
               30.7, 0.412, 14.67612, -1.0922752, 0.00550]

    input_t= [-0.5, 0.00071, -1.0571, 0.014]





    align_ctrl.solve(state_t, input_t)

    x = align_ctrl.x
    u = align_ctrl.u
    plot_align_result(x, u)




