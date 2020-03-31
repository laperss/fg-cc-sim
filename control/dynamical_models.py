#! /usr/bin/python
"""
Load different dynamical models of the vehicles. 
"""
from __future__ import print_function
import numpy as np
import math

psi_ugv = 0.7  # ugv steering time constant
a_ugv = [10.3784, -10.545599]
psi_uav = 0.2  # uav steering time constant
phi_uav = 3.0  # uav steering time constant
a_uav = [1.776, -1.776-0.1540]

coefficients = [2.57,    0.91,    0.74,    0.69,   -0.53]

roll_gain = coefficients[0]
roll_ctrl = coefficients[1]
yaw_gain = coefficients[2]
yaw_ctrl = coefficients[3]
yaw_damp = coefficients[4]


coefficients = [2.9052,    0.9723,    2.2720,    0.6457,    0.0422]
roll_ctrl = coefficients[0]
yaw_gain = coefficients[1]
roll_damp = coefficients[2]
yaw_ctrl = coefficients[3]
yaw_damp = coefficients[4]
acc_gain = 1.6
acc_damp = 0.0715


def horizontal_derivatives(states,inputs):

    N = states.shape[0]
    nx = states.shape[1]
    derivative = np.zeros((N,nx))

    psi_ugv = 0.7 # ugv steering time constant
    a_ugv = [10.3784, -10.545599]

    coefficients = [2.9052,    0.9723,    2.2720,    0.6457,    0.0422]
    roll_ctrl = coefficients[0]
    yaw_gain = coefficients[1]
    roll_damp = coefficients[2]
    yaw_ctrl = coefficients[3]
    yaw_damp = coefficients[4]
    acc_gain = 1.6
    acc_damp = 0.0715
    # initial derivative
    for i in range(N):
       derivative[i,0] = np.cos(states[i,4])*states[i,2]; 
       derivative[i,1] = np.sin(states[i,4])*states[i,2]; 
       derivative[i,2] = states[i,3]; 
       derivative[i,3] = (-acc_gain-acc_damp)*states[i,3] + acc_gain*inputs[i,0];
       derivative[i,4] = (yaw_ctrl-yaw_damp)*states[i,4] + yaw_gain*states[i,5] -yaw_ctrl*inputs[i,1];
       derivative[i,5] = -roll_ctrl*states[i,4] - roll_damp*states[i,5]+ roll_ctrl*inputs[i,1];
       
       derivative[i,6] = np.cos(states[i,10])*states[i,8]; 
       derivative[i,7] = np.sin(states[i,10])*states[i,8]; 
       derivative[i,8] = states[i,9]; 
       derivative[i,9] = a_ugv[1]*states[i,9] + a_ugv[0]*inputs[i,2]
       derivative[i,10] =  -psi_ugv*states[i,10] + psi_ugv*inputs[i,3]
    return derivative

def integrate_system(inputs, states, h):

    # Compute next state from state and input using RK4
    derivative = horizontal_derivatives(states.T,inputs.T);
    # k1 = h*f(x)
    k1 = derivative*h;
    y1 = states + k1.T/2;
    
    derivative = horizontal_derivatives(y1.T,inputs.T);
    # k2 = h*f(x+k1/2)
    k2 = h*derivative;
    y2 = states  + k2.T/2;
    
    derivative = horizontal_derivatives(y2.T,inputs.T);  
    # k3 = h*f(x+k2/2)
    k3 = h*derivative;
    y3 = states  + k3.T;
    
    derivative = horizontal_derivatives(y3.T,inputs.T);
    # k4 = h*f(x+k3)
    k4 = h*derivative;

    
    next_state = states  + 1/6*(k1.T + 2*k2.T + 2*k3.T + k4.T);

    #print(next_state)
    return next_state

def integrate_path(zvec):
    inputs = [zvec[0:15:-1], zvec[1:15:-1], zvec[2:15:-1], zvec[3:15:-1]]

    states = [zvec[4:15:-1], zvec[5:15:-1], zvec[6:15:-1], zvec[7:15:-1], zvec[8:15:-1], zvec[9:15:-1], 
              zvec[10:15:-1], zvec[11:15:-1], zvec[12:15:-1], zvec[13:15:-1], zvec[14:15:-1]]

    #N = size(states,1);

    # Compute next state from state and input using RK4
    derivative = horizontal_derivatives(states[1:N-1,:],inputs)
    # k1 = h*f(x)
    k1 = derivative*h;
    y1 = states[1:-2,:] + k1/2;
    
    derivative = horizontal_derivatives(y1,inputs)
    # k2 = h*f(x+k1/2)
    k2 = h*derivative;
    y2 = states[1:-2,:]  + k2/2;
    
    derivative = horizontal_derivatives(y2,inputs)
    # k3 = h*f(x+k2/2)
    k3 = h*derivative;
    y3 = states[1:-2,:]  + k3;
    
    derivative = horizontal_derivatives(y3,inputs)
    # k4 = h*f(x+k3)
    k4 = h*derivative;
    next_state = states[1:-2,:]  + 1/6*(k1 + 2*k2 + 2*k3 + k4)

    print(next_state)

def get_param(dt=None):
    A44 = psi_uav
    A45 = 0.55
    A54 = -phi_uav
    A55 = -2

    steps = 3
    # List of coefficients
    A14 = [0 for i in range(steps+1)]
    A15 = [0 for i in range(steps+1)]

    A14[0] = 1
    A15[0] = 0

    for i in range(1, steps):
        A14[i] = A14[i-1]*A44 + A15[i-1]*A54
        A15[i] = A14[i-1]*A45 + A15[i-1]*A55

    if dt is None:
        return [A14, A15]

    else:
        yparam_psi = sum([A14[i]*dt**(i+1)/math.factorial(i+1)
                          for i in range(steps+1)])
        yparam_phi = sum([A15[i]*dt**(i+1)/math.factorial(i+1)
                          for i in range(steps+1)])

    return [yparam_psi, yparam_phi]


def get_y_uav(dt=None):
    A44 = psi_uav
    A45 = 0.55
    A54 = -phi_uav
    A55 = -2

    # List of coefficients
    A14 = [0, 0, 0, 0]
    A15 = [0, 0, 0, 0]

    A14[0] = 1
    A15[0] = 0

    A14[1] = A44  # = A14[0]*A44 + A15[0]*A54
    A15[1] = A45  # = A14[0]*A45 + A15[0]*A55

    A14[2] = A44*A44 + A45*A54  # = A14[1]*A44 + A15[1]*A54
    A15[2] = A44*A45 + A45*A55  # = A14[1]*A45 + A15[1]*A55

    A14[3] = A44*A44*A44 + A45*A54*A44 + A44*A45*A54 + A45*A55*A54
    A15[3] = A44*A44*A45 + A45*A54*A45 + A44*A45*A55 + A45*A55*A55

    if dt is None:
        return [A14, A15]

    else:
        yparam_psi = sum([A14[i]*dt**(i+1)/math.factorial(i+1)
                          for i in range(len(A14))])
        yparam_phi = sum([A15[i]*dt**(i+1)/math.factorial(i+1)
                          for i in range(len(A15))])

    return [yparam_psi, yparam_phi]


def get_y_ugv(dt=None):
    A44 = -psi_ugv

    # List of coefficients
    A14 = [0, 0, 0, 0]

    A14[0] = 1

    A14[1] = A44  # = A14[0]*A44

    A14[2] = A44*A44  # = A14[1]*A44

    A14[3] = A44*A44*A44  # = A14[2]*A44

    if dt is None:
        return A14

    else:
        yparam = sum([A14[i]*dt**(i+1)/math.factorial(i+1)
                      for i in range(len(A14))])

    return yparam


def get_h_uav(dt=None):
    A11 = -1.607

    # List of coefficients
    A01 = [0, 0, 0, 0]

    A01[0] = 1

    A01[1] = A11  # = A14[0]*A44

    A01[2] = A11*A11  # = A14[1]*A44

    A01[3] = A11*A11*A11  # = A14[2]*A44

    if dt is None:
        return A01

    else:
        hparam = sum([A01[i]*dt**(i+1)/math.factorial(i+1)
                      for i in range(len(A01))])

    return hparam


def get_vertical_dynamics():
    A = np.array([[0.0, 22.5],
                  [0.0, -1.607]])
    B = np.array([[-4.858],
                  [1.607]])

    # A = np.array([[1., 3.925],
    #              [0., 0.746]])
    # B = np.array([[-0.2788],
    #              [0.2508]])
    C = np.concatenate((np.eye(2), np.zeros((1, 2))))
    D = np.array([[0], [0], [1]])
    return A, B, C, D


def get_horizontal_dynamics(v_ref):
    """
    x' = Ax + Bu
    y = Cx + Du
    Cost variables: Fx and Gu
    """

    # ----------   x  y  v   a    psi   phi  x  y  v   a  psi
    A = np.array([[0, 0, 1,  0,    0.0,  0.0, 0, 0, 0,  0, 0],  # x
                  [0, 0, 0,  0,    v_ref,  0.0, 0, 0, 0,  0, 0],  # y
                  [0, 0, 0,  1,    0.0,  0.0, 0, 0, 0,  0, 0],  # v
                  [0, 0, 0, -acc_gain-acc_damp, 0.0,  0.0, 0, 0, 0,  0, 0],  # a
                  [0, 0, 0,  0,  yaw_ctrl-yaw_damp,
                      yaw_gain, 0, 0, 0, 0, 0],  # psi
                  [0, 0, 0,  0,  -roll_ctrl, -roll_damp, 0, 0, 0,  0, 0.0],  # phi
                  [0, 0, 0,  0,    0.0,  0.0, 0, 0, 1,  0, 0.0],  # x
                  [0, 0, 0,  0,    0.0,  0.0, 0, 0, 0,  0, v_ref],  # y
                  [0, 0, 0,  0,    0.0,  0.0, 0, 0, 0,  1,  0.0],  # v
                  [0, 0, 0,  0,    0.0,  0.0, 0, 0, 0, a_ugv[1], 0.0],  # a
                  [0, 0, 0,  0,    0.0,  0.0, 0, 0, 0,   0.0, -psi_ugv]])

    # -----------   T            S      T      S
    B = np.array([[0.0,        0.0,       0.0,       0.0],  # x
                  [0.0,        0.0,       0.0,       0.0],  # y
                  [0.0,        0.0,       0.0,       0.0],  # v
                  [acc_gain,    0.0,      0.0,      0.0],  # a
                  [0.0,   -yaw_ctrl,      0.0,       0.0],  # psi
                  [0.0,    roll_ctrl,     0.0,      0.0],  # phi
                  [0.0,         0.0,      0.0,       0.0],  # x
                  [0.0,         0.0,      0.0,       0.0],  # y
                  [0.0,         0.0,      0.0,       0.0],  # v
                  [0.0,         0.0,      a_ugv[0],  0.0],  # a
                  [0.0,         0.0,      0.0,  psi_ugv]])  # psi

    # -----------   T            S      T      S
    Bd = np.array([[0.0,        0.0,       0.0,       0.0],  # x
                   [0.0,        0.0,       0.0,       0.0],  # y
                   [0.0,        0.0,       0.0,       0.0],  # v
                   [1.0,        0.0,      0.0,      0.0],  # a
                   [0.0,        0.1,      0.0,       0.0],  # psi
                   [0.0,        1.0,       0.0,      0.0],  # phi
                   [0.0,         0.0,      0.0,       0.0],  # x
                   [0.0,         0.0,      0.0,       0.0],  # y
                   [0.0,         0.0,      0.0,       0.0],  # v
                   [0.0,         0.0,      1.0,      0.0],  # a
                   [0.0,         0.0,      0.0,      1.0]])  # psi

    # ----------------  x  y  v  a  ps ph x  y  v  a  ps
    C = np.array([[0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],   # v_uav
                  [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],   # v_ugv
                  [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],   # a_uav
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],   # a_ugv
                  [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],   # phi_uav
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
    C = np.array([[0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],   # v_uav
                  [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],   # v_ugv
                  [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],   # a_uav
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],   # a_ugv
                  [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],   # phi_uav
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

    D = np.array([[0, 0, 0, 0],
                  [0, 0, 0, 0],
                  [0, 0, 0, 0],
                  [0, 0, 0, 0],
                  [0, 0, 0, 0],
                  [1, 0, 0, 0],   # thrust_uav
                  [0, 1, 0, 0],   # steering_uav
                  [0, 0, 1, 0],   # thrust_ugv
                  [0, 0, 0, 1]])   # steering_ugv

    # F = np.zeros((0, 11))  # Remove state cost

    return A, B, C, D, Bd
