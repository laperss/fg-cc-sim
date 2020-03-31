import numpy as np
from .Polyhedron import Polyhedron
from .LQR import *


def get_mpc_sets():
    Y_vrt = np.eye(3)
    Y_vrt_lb = np.array([[1, -0.30, -0.25]]).T
    Y_vrt_ub = np.array([[40, 0.30,  0.25]]).T

    F_vrt = np.eye(2)
    F_vrt_lb = np.array([[-1.0, -0.15]]).T
    F_vrt_ub = np.array([[0.4,   0.25]]).T

    W_vrt = Polyhedron(np.eye(2),
                       ub=np.array([[0.01, 0.002]]).T,
                       lb=-np.array([[0.01, 0.002]]).T,
                       name='Vertical disturbance set')
    Fset_vrt = Polyhedron(F_vrt, lb=F_vrt_lb, ub=F_vrt_ub, name='Terminal set')
    Yset_vrt = Polyhedron(Y_vrt, lb=Y_vrt_lb, ub=Y_vrt_ub,
                          name='Constraint set')

    # ======================= HORIZONTAL SETS ========================
    hrz_ref = np.array([[0, 0, 20.0, 0, 0, 0, 0, 0, 20.0, 0, 0]]).T

    # ------------------  x  y  v  a ps ph  x   y   v  a ps
    H_hrz = np.array([[1, 0, 0, 0, 0, 0, -1,  0,  0, 0, 0],   # deltax
                      [0, 1, 0, 0, 0, 0,  0, -1,  0, 0, 0],   # deltay
                      [0, 0, 1, 0, 0, 0,  0,  0, -1, 0, 0],   # deltav
                      [0, 0, 1, 0, 0, 0,  0,  0,  0, 0, 0],   # vuav
                      [0, 0, 0, 0, 0, 0,  0,  0,  1, 0, 0],   # vugv
                      [0, 0, 0, 1, 0, 0,  0,  0,  0, 0, 0],   # a_uav
                      [0, 0, 0, 0, 0, 0,  0,  0,  0, 1, 0],   # a_ugv
                      [0, 0, 0, 0, 1, 0,  0,  0,  0, 0, 0],   # psi_uav
                      [0, 0, 0, 0, 0, 1,  0,  0,  0, 0, 0],   # phi_uav
                      [0, 0, 0, 0, 0, 0,  0,  0,  0, 0, 1]])   # psi_ugv
    # ------------------  x  y  v  a ps ph  x   y   v  a ps
    H_hrz_new = np.array([[1, 0, 0, 0, 0, 0, -1,  0,  0, 0, 0],   # deltax
                          [0, 1, 0, 0, 0, 0,  0, -1,  0, 0, 0],   # deltay
                          [0, 0, 1, 0, 0, 0,  0,  0, -1, 0, 0],   # deltav
                          [0, 0, 0, 1, 0, 0,  0,  0,  0, 0, 0],   # a_uav
                          [0, 0, 0, 0, 0, 0,  0,  0,  0, 1, 0],   # a_ugv
                          [0, 0, 0, 0, 1, 0,  0,  0,  0, 0, 0],   # psi_uav
                          [0, 0, 0, 0, 0, 1,  0,  0,  0, 0, 0],   # phi_uav
                          [0, 0, 0, 0, 0, 0,  0,  0,  0, 0, 1]])   # psi_ugv
    G_hrz = np.array([[1, 0, 0, 0],   # thrust_uav
                      [0, 1, 0, 0],   # steering_ugv
                      [0, 0, 1, 0],   # thrust_ugv
                      [0, 0, 0, 1]])  # steering_uav


    Y_lb = np.array([[17,      # UAV Velocity
                      0,       # UGV Velocity
                      -1.5,    # UAV Acceleration
                      -3.8,    # UGV Acceleration
                      -0.30,   # UAV Roll
                      -0.8,    # UAV Acceleration input
                      -0.2,   # UAV Yawrate input
                      -3.5,    # UGV Acceleration input
                      -0.6]]).T

    Y_ub = np.array([[25,
                      25,
                      2.0,
                      3.8,
                      0.20,
                      1.0,
                      0.15,
                      3.5,
                      0.6]]).T
    Y_hrz = np.eye(len(Y_lb))

    F_hrz = np.array([[1, 0,  0,  0,  0, 0, -1,  0,  0,  0,  0],    # deltax
                      [0, 1,  0,  0,  0, 0,  0, -1,  0,  0,  0],    # deltay
                      [0, 0,  1,  0,  0, 0,  0,  0, -1,  0,  0],    # deltav
                      [0, 0,  0,  1,  0, 0,  0,  0,  0, -1,  0],    # deltaa
                      [0, 0,  0,  0,  1, 0,  0,  0,  0,  0, -1]])   # delta_psi

    F_lb = np.array([[-0.5, -0.9, -0.1, -0.2, -0.31415]]).T
    F_ub = np.array([[0.5,  0.9,  0.1,  0.2, 0.31415]]).T

    w_ub = np.array([[0, 0, 0, 0.05*0.1,   0.03*0.1, 0,
                      0, 0, 0, 0.05*0.1,  0.0003*0.1]]).T
    w_lb = - w_ub
    W = Polyhedron(np.eye(11), ub=w_ub, lb=w_lb, name='Disturbance set')

    # w_ub2 = np.abs(np.matmul(Bd_hrz, np.array(
    #    [[0.05,   0.05, 0.05,  0.0005]]).T))
    #w_lb2 = - w_ub2
    #W = Polyhedron(np.eye(11), ub=w_ub2, lb=w_lb2, name='Disturbance set')

    #print(np.concatenate((W2.b, W.b), axis=1))
    np.set_printoptions(precision=3, linewidth=250,
                        edgeitems=8, suppress=True)

    Fset = Polyhedron(F_hrz, lb=F_lb, ub=F_ub, name='Terminal set')
    Yset = Polyhedron(Y_hrz, lb=Y_lb, ub=Y_ub, name='Constraint set')

    return Fset_vrt, Yset_vrt, W_vrt, Fset, Yset, W


def get_mpc_costs(A_hrz, B_hrz, Bd_hrz, A_vrt, B_vrt, v_ref):
    Q_vrt = np.array([[2, 0],
                      [0, 5*180/np.pi]])
    Q_vrt = np.array([[2, 0],
                      [0, 0*180/np.pi]])
    R_vrt = np.array([[15*180/np.pi]])

    # ======================= HORIZONTAL SETS ========================
    hrz_ref = np.array([[0, 0, v_ref, 0, 0, 0, 0, 0, v_ref, 0, 0]]).T

    # ------------------  x  y  v  a ps ph  x   y   v  a ps
    H_hrz = np.array([[1, 0, 0, 0, 0, 0, -1,  0,  0, 0, 0],   # deltax
                      [0, 1, 0, 0, 0, 0,  0, -1,  0, 0, 0],   # deltay
                      [0, 0, 1, 0, 0, 0,  0,  0, -1, 0, 0],   # deltav
                      [0, 0, 1, 0, 0, 0,  0,  0,  0, 0, 0],   # vuav
                      [0, 0, 0, 0, 0, 0,  0,  0,  1, 0, 0],   # vugv
                      [0, 0, 0, 1, 0, 0,  0,  0,  0, 0, 0],   # a_uav
                      [0, 0, 0, 0, 0, 0,  0,  0,  0, 1, 0],   # a_ugv
                      [0, 0, 0, 0, 1, 0,  0,  0,  0, 0, 0],   # psi_uav
                      [0, 0, 0, 0, 0, 1,  0,  0,  0, 0, 0],   # phi_uav
                      [0, 0, 0, 0, 0, 0,  0,  0,  0, 0, 1]])   # psi_ugv
    # ------------------  x  y  v  a ps ph  x   y   v  a ps
    H_hrz_new = np.array([[1, 0, 0, 0, 0, 0, -1,  0,  0, 0, 0],   # deltax
                          [0, 1, 0, 0, 0, 0,  0, -1,  0, 0, 0],   # deltay
                          [0, 0, 1, 0, 0, 0,  0,  0, -1, 0, 0],   # deltav
                          [0, 0, 0, 1, 0, 0,  0,  0,  0, 0, 0],   # a_uav
                          [0, 0, 0, 0, 0, 0,  0,  0,  0, 1, 0],   # a_ugv
                          [0, 0, 0, 0, 1, 0,  0,  0,  0, 0, 0],   # psi_uav
                          [0, 0, 0, 0, 0, 1,  0,  0,  0, 0, 0],   # phi_uav
                          [0, 0, 0, 0, 0, 0,  0,  0,  0, 0, 1]])   # psi_ugv
    G_hrz = np.array([[1, 0, 0, 0],   # thrust_uav
                      [0, 1, 0, 0],   # steering_ugv
                      [0, 0, 1, 0],   # thrust_ugv
                      [0, 0, 0, 1]])  # steering_uav
    # ----------------- dx   dy   dv   v    v    a   a   ps   ph   ps
    Q_hrz = np.array([[5.0, 0.0, 0.0,  0,   0,   0,   0,   0,   0,   0],
                      [0.0, 0.5, 0.0,  0,   0,   0,   0,   0,   0,   0],
                      [0.0, 0.0, 2.0,  0,   0,   0,   0,   0,   0,   0],
                      [0.0, 0.0, 0.0,  3,   0,   0,   0,   0,   0,   0],
                      [0.0, 0.0, 0.0,  0,   3,   0,   0,   0,   0,   0],
                      [0.0, 0.0, 0.0,  0,   0,   5,   0,   0,   0,   0],
                      [0.0, 0.0, 0.0,  0,   0,   0,   10,   0,   0,   0],
                      [0.0, 0.0, 0.0,  0,   0,   0,   0, 150,   0,   0],
                      [0.0, 0.0, 0.0,  0,   0,   0,   0,   0,  80,  0],
                      [0.0, 0.0, 0.0,  0,   0,   0,   0,   0,   0,  150]])

    Q_hrz_new = np.array([[5.0, 0.0, 0.0,  0,   0,   0,   0,   0],
                          [0.0, 0.5, 0.0,   0,   0,   0,   0,   0],
                          [0.0, 0.0, 2.0,   0,   0,   0,   0,   0],
                          [0.0, 0.0, 0.0,   5,   0,   0,   0,   0],
                          [0.0, 0.0, 0.0,   0,   5,   0,   0,   0],
                          [0.0, 0.0, 0.0,   0,   0, 350,   0,   0],
                          [0.0, 0.0, 0.0,   0,   0,   0,  200,  0],
                          [0.0, 0.0, 0.0,   0,   0,   0,   0,  200]])
    R_hrz = np.array([[20, 0,  0,  0],
                      [0, 150, 0,  0],
                      [0, 0, 30,  0],
                      [0, 0,  0, 150]])  # *0.001

    Q_hrz = np.matmul(H_hrz.T, np.matmul(Q_hrz, H_hrz))
    R_hrz = np.matmul(G_hrz.T, np.matmul(R_hrz, G_hrz))

    q_hrz = -np.matmul(Q_hrz, hrz_ref)
    r_hrz = np.zeros((4, 1))

    #const_hrz = 250*0.5*np.matmul(hrz_ref.T, np.matmul(Q_hrz, hrz_ref))

    # Compute terminal cost
    LQRgain, Qf, qf = get_lqr_feedback(A_hrz, B_hrz, Q_hrz, R_hrz, q_hrz)
    qf_hrz = -np.matmul(Qf, hrz_ref)

    
    const_hrz = 0.5*np.matmul(hrz_ref.T, np.matmul(Qf, hrz_ref))
    #print("======== COSNT ===========")
    #print(const_hrz)

    #print("qf compare: ")
    #print(qf_hrz.T)
    #print(qf.T)

    return Q_vrt, R_vrt, Q_hrz, R_hrz, q_hrz, r_hrz, Qf, qf_hrz, const_hrz
