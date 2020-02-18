import numpy as np
from .Polyhedron import Polyhedron

Q_vrt = np.array([[1, 0], [0, 5*180/np.pi]])
R_vrt = np.array([[15*180/np.pi]])

Y_vrt = np.eye(3)
Y_vrt_lb = np.array([[1, -np.pi/7, -np.pi/8]]).T
Y_vrt_ub = np.array([[40, np.pi/6, np.pi/7]]).T

F_vrt = np.eye(2)
F_vrt_lb = np.array([[-0.5, -0.2*np.pi]]).T
F_vrt_ub = np.array([[0.5,   0.2*np.pi]]).T

W_vrt = Polyhedron(np.eye(2),
                   ub=np.array([[0.01, 0.002]]).T,
                   lb=-np.array([[0.01, 0.002]]).T,
                   name='Vertical disturbance set')
Fset_vrt = Polyhedron(F_vrt, lb=F_vrt_lb, ub=F_vrt_ub, name='Terminal set')
Yset_vrt = Polyhedron(Y_vrt, lb=Y_vrt_lb, ub=Y_vrt_ub, name='Constraint set')


# ----------------- dx   dy   dv   v    v    a   a   ps   ph   ps
Q_hrz = np.array([[5.0, 0.0, 0.0,  0,   0,   0,   0,   0,   0,   0],
                  [0.0, 0.5, 0.0,  0,   0,   0,   0,   0,   0,   0],
                  [0.0, 0.0, 2.0,  0,   0,   0,   0,   0,   0,   0],
                  [0.0, 0.0, 0.0,  3,   0,   0,   0,   0,   0,   0],
                  [0.0, 0.0, 0.0,  0,   3,   0,   0,   0,   0,   0],
                  [0.0, 0.0, 0.0,  0,   0,   5,   0,   0,   0,   0],
                  [0.0, 0.0, 0.0,  0,   0,   0,   5,   0,   0,   0],
                  [0.0, 0.0, 0.0,  0,   0,   0,   0,  50,   0,   0],
                  [0.0, 0.0, 0.0,  0,   0,   0,   0,   0,  80,  0],
                  [0.0, 0.0, 0.0,  0,   0,   0,   0,   0,   0,  250]])

# ----------------- dx   dy   dv     v1    v2    a    a   ps   ph   ps
q_hrz = np.array(
    [[0.0, 0.0, 0.0, -3*20.0, -3*20.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T

R_hrz = np.array([[10, 0,  0,  0],
                  [0, 1, 0,  0],
                  [0, 0,  50,  0],
                  [0, 0,  0, 50]])

r_hrz = np.array([[0.0, 0.0, 0.0, 0.0]]).T


Y_lb = np.array([[17,
                  0,
                  -2.0,
                  -3.5,
                  -0.20,
                  -2.0,
                  -0.15,
                  -3.5,
                  -0.6]]).T

Y_ub = np.array([[25,
                  25,
                  2.0,
                  3.5,
                  0.20,
                  2.0,
                  0.15,
                  3.5,
                  0.6]]).T
Y_hrz = np.eye(len(Y_lb))

F_hrz = np.array([[1, 0,  0,  0,  0, 0, -1,  0,  0,  0,  0],    # deltax
                  [0, 1,  0,  0,  0, 0,  0, -1,  0,  0,  0],    # deltay
                  [0, 0,  1,  0,  0, 0,  0,  0, -1,  0,  0],    # deltav
                  [0, 0,  0,  0,  1, 0,  0,  0,  0,  0, -1]])   # delta_psi

F_lb = np.array([[-1.3, -2.9, -0.3, -0.1*np.pi]]).T
F_ub = np.array([[1.3,  2.9,  0.3,  0.1*np.pi]]).T

w_ub = np.array([[0, 0, 0, 0.4*0.1, 0.01*np.pi*0.1, 0,
                  0, 0, 0, 0.1*0.1, 0.001*np.pi*0.1]]).T
w_lb = - w_ub

np.set_printoptions(precision=3, linewidth=250,
                    edgeitems=8, suppress=True)


Fset = Polyhedron(F_hrz, lb=F_lb, ub=F_ub, name='Terminal set')
Yset = Polyhedron(Y_hrz, lb=Y_lb, ub=Y_ub, name='Constraint set')
W = Polyhedron(np.eye(11), ub=w_ub, lb=w_lb, name='Disturbance set')
