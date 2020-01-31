import numpy as np
from .Polyhedron import Polyhedron

Q_vrt = np.array([[1, 0], [0, 100]])
R_vrt = np.array([[150]])

Y_vrt = np.eye(3)
Y_vrt_lb = np.array([[1, -np.pi/6, -np.pi/7]]).T
Y_vrt_ub = np.array([[40, np.pi/6, np.pi/7]]).T

F_vrt = np.eye(2)
F_vrt_lb = np.array([[-0.5, -0.2*np.pi]]).T
F_vrt_ub = np.array([[0.5,   0.2*np.pi]]).T

W_vrt = Polyhedron(np.eye(2),
                   ub=np.array([[0.01, 0.001]]).T,
                   lb=-np.array([[0.01, 0.001]]).T,
                   name='Vertical disturbance set')
Fset_vrt = Polyhedron(F_vrt, lb=F_vrt_lb, ub=F_vrt_ub, name='Terminal set')
Yset_vrt = Polyhedron(Y_vrt, lb=Y_vrt_lb, ub=Y_vrt_ub, name='Constraint set')


# -----------------    dx   dy   dv  a   a   ps   ph ps
Q_hrz = np.matrix([[2.0, 0.0, 0.0,  0,  0,   0,   0,   0],
                   [0.0, 2.0, 0.0,  0,  0,   0,   0,   0],
                   [0.0, 0.0, 1.0,  0,  0,   0,   0,   0],
                   [0.0, 0.0, 0.0, 10,  0,   0,   0,   0],
                   [0.0, 0.0, 0.0,  0, 10,   0,   0,   0],
                   [0.0, 0.0, 0.0,  0,  0,  50,   0,   0],
                   [0.0, 0.0, 0.0,  0,  0,   0,  80,   0],
                   [0.0, 0.0, 0.0,  0,  0,   0,   0,  250]])

R_hrz = np.matrix([[1, 0,  0,  0],
                   [0, 10, 0,  0],
                   [0, 0,  1,  0],
                   [0, 0,  0, 20]])


Y_hrz = np.eye(10)
Y_lb = np.array([[15, 0, *[-3]*2, *[-np.pi/5]*2, -4.5, -np.pi/6, -5.5,  -np.pi/6]]).T
Y_ub = np.array([[25, 25, *[3]*2, *[np.pi/5]*2, 4.5, np.pi/6, 5.5,  np.pi/6]]).T

F_hrz = np.array([[1, 0,  0,  0,  0, 0, -1,  0,  0,  0,  0],    # deltax
                  [0, 1,  0,  0,  0, 0,  0, -1,  0,  0,  0],    # deltay
                  [0, 0,  1,  0,  0, 0,  0,  0, -1,  0,  0],    # deltav
                  [0, 0,  0,  0,  1, 0,  0,  0,  0,  0, -1]])   # delta_psi

F_lb = np.array([[-1.5, -2.5, -0.8, -0.3*np.pi]]).T
F_ub = np.array([[1.5,  2.5,  0.8,  0.3*np.pi]]).T

w_ub = np.array([[0, 0, 0, 0.01, 0.0001*np.pi, 0,
                  0, 0, 0, 0.01, 0.0001*np.pi]]).T
w_lb = - w_ub

np.set_printoptions(precision=3, linewidth=250,
                    edgeitems=8, suppress=True)


Fset = Polyhedron(F_hrz, lb=F_lb, ub=F_ub, name='Terminal set')
Yset = Polyhedron(Y_hrz, lb=Y_lb, ub=Y_ub, name='Constraint set')
W = Polyhedron(np.eye(11), ub=w_ub, lb=w_lb, name='Disturbance set')
