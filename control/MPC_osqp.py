import numpy as np
import osqp
import scipy
import scipy.linalg
import scipy.io as sio
from scipy.sparse import csc_matrix, block_diag
import matplotlib.pyplot as plt
import utils
from .Polyhedron import Polyhedron
import time


class ControllerOSQP(object):
    """ A class for executing the model predictive control"""

    def __init__(self, A, B, C, D, N, ds, nt=0):
        self.N = N
        self.ds = ds
        self.A = A
        self.B = B
        self.C = C
        self.D = D

        self.nx = A.shape[0]
        self.nu = B.shape[1]
        self.ny = C.shape[0]             # number of outputs
        self.nz = N*(self.nx + self.nu)  # total number of variables
        self.ni = N*self.ny              # number of inequalities
        self.ne = N*self.nx              # number of equalities
        self.nt = nt                      # number of terminal constraints
        self.nc = self.ni + self.ne + self.nt  # total number of constraints

        self.x0 = None
        self.y0 = None

        self.cost = np.zeros((self.nz, self.nz))

        # The equality matrix:
        # 1:ne = equality constraints
        # ne+1:ne+ni: inequality constraints
        # ne+ni+1:ne+ne+nt: terminal constraints.
        self.ineq_A = np.zeros((self.nc, self.nz))
        self.ineq_l = np.zeros((self.nc, 1))
        self.ineq_u = np.zeros((self.nc, 1))

        self.set_equality_constraints(A, B)

        self.settings = {'verbose': False,
                         'time_limit': 0.05, 'eps_abs': 1e-4, 'eps_rel': 1e-4}

        self.optimizer = osqp.OSQP()

    def set_cost_matrix(self, Q, R, Qf, F=None, G=None):
        self.C2 = F
        self.D2 = G
        if F is not None:
            Q = np.matmul(F.T, np.matmul(Q, F))
            Qf = np.matmul(F.T, np.matmul(Qf, F))

        if G is not None:
            R = np.matmul(G.T, np.matmul(R, G))

        if self.N > 1:
            for i in range(1, self.N):
                cost = block_diag([R, scipy.sparse.kron(scipy.sparse.eye(self.N-1),
                                                        block_diag([Q, R])), Qf], format='csc')
        self.cost = cost

    def set_equality_constraints(self, A, B):
        ineq_A = np.zeros((self.ne, self.nz))
        ineq_l = np.zeros((self.ne, 1))
        ineq_A[0: self.nx, 0: self.nu] = -B
        ineq_A[0: self.nx, self.nu: self.nu+self.nx] = np.eye(self.nx)
        ineq_l[0:self.nx, 0] = 1
        for i in range(1, self.N):
            ineq_A[i*self.nx: (i+1)*self.nx, i*self.nu+(i-1) *
                   self.nx:i*self.nu+i*self.nx] = -A
            ineq_A[i*self.nx: (i+1)*self.nx, i*self.nu+(i) *
                   self.nx: (i+1)*self.nu + i*self.nx] = -B
            ineq_A[i*self.nx: (i+1)*self.nx,  (i+1)*self.nu+i *
                   self.nx: (i+1)*self.nu+(i+1)*self.nx] = np.eye(self.nx)
        self.ineq_A[0:self.ne, :] = ineq_A
        self.ineq_l[0:self.ne, :] = ineq_l
        self.ineq_u[0:self.ne, :] = ineq_l

    def update_equality_constraints(self, row, column, values):
        ineq_A = self.ineq_A[0:self.ne, :]
        ineq_l = self.ineq_l[0:self.ne, :]

        for i in range(1, self.N):
            ineq_A[i*self.nx+row, i*self.nu +
                   (i-1)*self.nx+column] = -values[i]

        self.ineq_A[0:self.ne, :] = ineq_A
        self.ineq_l[0:self.ne, :] = ineq_l
        self.ineq_u[0:self.ne, :] = ineq_l

    def set_inequality_constraints(self, Y):
        print("* Number of inequality constraints = %i" % self.ni)
        # Initial constraint
        idxu = np.where(np.any((Y.P @ self.D), axis=1))[0]
        constr_0 = ((Y.P @ self.D))[idxu]
        lb_0 = Y.lb[idxu, :]
        ub_0 = Y.ub[idxu, :]

        # Constraint t=1:N-1
        ub_i = np.tile(Y.ub, (self.N-1, 1))
        lb_i = np.tile(Y.lb, (self.N-1, 1))
        constr_i = scipy.sparse.kron(np.eye(self.N-1),
                                     (Y.P @ np.concatenate((self.C,
                                                            self.D), axis=1)), format='csc')
        # Terminal constraint
        idxx = np.where(np.any((Y.P @ self.C), axis=1))[0]
        constr_N = ((Y.P @ self.C))[idxx, :]
        lb_N = Y.lb[idxx, :]
        ub_N = Y.ub[idxx, :]

        ineq_A = block_diag(
            [constr_0, constr_i.todense(), constr_N], format='csc')

        lb = np.vstack([lb_0, lb_i, lb_N])
        ub = np.vstack([ub_0, ub_i, ub_N])

        self.ineq_A[self.ne:self.ne+self.ni, :] = ineq_A.todense()
        self.ineq_l[self.ne:self.ne+self.ni, :] = lb
        self.ineq_u[self.ne:self.ne+self.ni, :] = ub

    def set_terminal_constraint(self, F):
        print("* Number of terminal constraints = %i" % self.nt)
        print("N = ", self.N)

        Xf = F.P.todense()
        xl = F.lb
        xu = F.ub

        ineq_A = np.zeros((self.nt, self.nz))
        ineq_l = np.zeros((self.nt, 1))
        ineq_u = np.zeros((self.nt, 1))

        ineq_A[0: self.nt, self.N*self.nu +
               (self.N-1)*self.nx: self.nz] = Xf
        ineq_l = xl  # Lower bound
        ineq_u = xu  # Upper bound

        self.ineq_A[self.ne+self.ni:self.ne +
                    self.ni+self.nt, :] = ineq_A
        self.ineq_l[self.ne+self.ni:self.ne+self.ni+self.nt, :] = ineq_l
        self.ineq_u[self.ne+self.ni:self.ne+self.ni+self.nt, :] = ineq_u

    def setup_problems(self):
        self.ineq_A = csc_matrix(self.ineq_A)
        self.optimizer.setup(P=self.cost, q=np.zeros((self.nz, 1)),
                             A=self.ineq_A, l=self.ineq_l, u=self.ineq_u, **self.settings)
        res = self.optimizer.solve()

    def solve(self, x0, lb=None, u0=[0, 0, 0, 0], distance=None, k=0):

        # x0 = np.array([x0]).T
        Ax0 = np.matmul(self.A, np.array([x0]).T)
        # print("x0 = ", Ax0.T)

        if k > 0:
            if lb is None:
                start = time.time()
                lb = self.ineq_l.copy()
                ub = self.ineq_u.copy()
                lb[0: k * self.nx] = np.zeros((k*self.nx, 1))
                ub[0: k * self.nx] = np.zeros((k*self.nx, 1))
                lb[self.ne: self.ne+k * self.ny] = np.zeros((k*self.ny, 1))
                ub[self.ne: self.ne+k * self.ny] = np.zeros((k*self.ny, 1))

                lb[k*self.nx: k*self.nx + self.nx] = Ax0
                ub[k*self.nx: k*self.nx + self.nx] = Ax0
                self.optimizer.update(l=lb, u=ub)
                print("UPDATE TIME = ", time.time() - start)
                # print(lb[self.ne:])
                # print(ub[self.ne:])

            else:
                lb[0: k] = np.zeros(k)
                ub = self.ineq_u.copy()
                lb[0: k * self.nx] = np.zeros((k*self.nx, 1))
                ub[0: k * self.nx] = np.zeros((k*self.nx, 1))
                lb[self.ne: self.ne+k * self.ny] = np.zeros((k*self.ny, 1))
                ub[self.ne: self.ne+k * self.ny] = np.zeros((k*self.ny, 1))

                lb[k: k + self.nx] = Ax0
                ub[k: k + self.nx] = Ax0

                self.optimizer.update(l=lb, u=ub)

        else:
            if lb is None:
                self.ineq_l[0: self.nx] = Ax0
                self.ineq_u[0: self.nx] = Ax0

                self.optimizer.update(l=self.ineq_l, u=self.ineq_u)
            else:
                lb[0: self.nx] = Ax0
                self.ineq_u[0: self.nx] = Ax0

                self.optimizer.update(l=lb, u=self.ineq_u)

        start = time.time()

        res = self.optimizer.solve()
        print("SOLUTION TIME  ", self.nx,  time.time() - start)
        if (res.info.status_val == 1 or res.info.status_val == 2):
            pass
        else:
            print("NO ACCURATE SOLUTION FOUND")

        self.x0 = res.x
        self.y0 = res.y

        return res.x, res.info.status_val


def reachability_matrices(A, B, C, D, W, Xf, Y0, p, nmax):
    """ Compute the worst-case disturbances with a linear feedback.
    The feedback is nilpotent in p steps.

    Returns: Y = list of state/input constraints
             Q = list of terminal constriants
    """
    nx = A.shape[0]
    nu = B.shape[1]

    K = utils.nilpotent_feedback(A, B, p)

    Q = [None for i in range(p+1)]  # from 0 to N
    Y = [None for i in range(p+1)]  # from 0 to N
    L = [None for i in range(p+1)]  # from 0 to N

    L[0] = np.eye(nx)
    Q[0] = Xf
    Y[0] = Y0

    print(W)
    print(Xf)
    print(Y0)

    for i in range(p):

        L[i+1] = np.matmul((A + np.matmul(B, K[i])), L[i])
        q, r = np.linalg.qr(L[i+1])
        x_to_y = np.matmul((C+np.matmul(D, K[i])), L[i])
        Y[i + 1] = Y[i].pontryagin_difference(W.affine_map(x_to_y),
                                              name="Y[%i]" % (i+1))
        Y[i+1].minrep()

        Q[i+1] = Q[i].pontryagin_difference(W.affine_map(L[i], name='W[%i]' % (i)),
                                            name="Q[%i]" % (i+1))
        Q[i+1].minrep()

    return Y, Q


class ControllerOSQPRobust(ControllerOSQP):
    def reachability_matrices(self, W, Xf, Y0, p):
        """ Compute the worst-case disturbances with a linear feedback.
        The feedback is nilpotent in p steps.

        Returns: Y = list of state/input constraints
        Q = list of terminal constriants
        """
        K = utils.nilpotent_feedback(self.A, self.B, p)

        Q = [None for i in range(p+1)]  # from 0 to N
        Y = [None for i in range(p+1)]  # from 0 to N
        L = [None for i in range(p+1)]  # from 0 to N

        L[0] = np.eye(self.nx)
        Q[0] = Xf
        Y[0] = Y0

        for i in range(p):

            L[i+1] = np.matmul((self.A + np.matmul(self.B, K[i])), L[i])
            q, r = np.linalg.qr(L[i+1])
            x_to_y = np.matmul((self.C+np.matmul(self.D, K[i])), L[i])
            Y[i + 1] = Y[i].pontryagin_difference(W.affine_map(x_to_y),
                                                  name="Y[%i]" % (i+1))
            Y[i+1].minrep()

            Q[i+1] = Q[i].pontryagin_difference(W.affine_map(L[i], name='W[%i]' % (i)),
                                                name="Q[%i]" % (i+1))
            Q[i+1].minrep()

        # self.set_inequality_constraints(Y)
        # self.set_terminal_constraint(Q)
        return Y, Q

    def set_inequality_constraints(self, Y):
        print("* Number of inequality constraints = %i" % self.ni)
        # Initial constriant
        idxu = np.where(
            np.any((Y[0].P @ self.D), axis=1))[0]
        constr_0 = (np.matmul(Y[0].P.todense(), self.D))[idxu]
        lb_0 = Y[0].lb[idxu, :]
        ub_0 = Y[0].ub[idxu, :]

        # Constraint t=1:N-1
        temp_Y = Y[1:-1].copy()
        temp_Y.extend([*[Y[-1]]*(self.N-len(Y)+1)])
        constr_i = block_diag([(Yi.P @ np.concatenate((self.C,
                                                       self.D), axis=1)) for Yi in temp_Y])
        lb_i = np.vstack([Yi.lb for Yi in temp_Y])
        ub_i = np.vstack([Yi.ub for Yi in temp_Y])

        idxx = np.where(
            np.any((Y[-1].P @ self.C), axis=1))[0]
        constr_N = ((Y[-1].P @ self.C))[idxx, :]
        lb_N = Y[-1].lb[idxx, :]
        ub_N = Y[-1].ub[idxx, :]

        ineq_A = block_diag(
            [constr_0, constr_i.todense(), constr_N], format='csc')

        lb = np.vstack([lb_0, lb_i, lb_N])
        ub = np.vstack([ub_0, ub_i, ub_N])

        self.ineq_A[self.ne:self.ne+self.ni, :] = ineq_A.todense()
        self.ineq_l[self.ne:self.ne+self.ni, :] = lb
        self.ineq_u[self.ne:self.ne+self.ni, :] = ub

    def set_terminal_constraint(self, F):
        print("* Number of terminal constraints = %i" % self.nt)
        self.F = F

        if type(F) != list:
            raise TypeError("F must be a list of sets.")

        if len(F) < self.N:
            F = F[-1]
        else:
            F = F[self.N]

        super().set_terminal_constraint(F)


class ControllerOSQPRobustVariableHorizon(ControllerOSQPRobust):
    def __init__(self, A, B, C, D, N, ds, nt=0):
        super().__init__(A, B, C, D, N, ds, nt)
        print("N = ", self.N)
        self.N0 = int(self.N/2)
        print("N = ", self.N)
        print("N0 = ", self.N0)

    def solve_N2(self, x0, N=0, lb=None):
        print("SOLVE N = ", N)
        Ax0 = np.matmul(self.A, np.array([x0]).T)

        nz = N*(self.nx + self.nu)
        neq = N*(self.nx + self.ny) + self.nt

        start = time.time()

        start = time.time()
        lb = np.zeros_like(self.ineq_l)
        ub = np.zeros_like(self.ineq_l)

        ineq_idx = [i for i in range(0, N*(self.nx))] + \
                   [self.ne + i for i in range(0, N*self.ny+self.nt)]

        lb[(self.N-N)*self.nx:self.N*self.nx, 0] = self.ineq_l[0:N*self.nx, 0]

        ub[(self.N-N)*self.nx:self.N*self.nx, 0] = self.ineq_u[0:N*self.nx, 0]

        lb[self.ne+(self.N-N)*self.ny:self.ne+self.N*self.ny,
           0] = self.ineq_l[self.ne:self.ne+N*self.ny, 0]

        ub[self.ne+(self.N-N)*self.ny:self.ne+self.N*self.ny,
           0] = self.ineq_u[self.ne:self.ne+N*self.ny, 0]

        lb[self.ne+self.ni:, 0] = self.ineq_l[self.ne+self.ni:, 0]

        ub[self.ne+(self.N-N)*self.ny:self.ne+self.N*self.ny,
           0] = self.ineq_u[self.ne:self.ne+N*self.ny, 0]

        lb[(self.N-N)*self.nx: (self.N-N)*self.nx + self.nx] = Ax0
        ub[(self.N-N)*self.nx: (self.N-N)*self.nx + self.nx] = Ax0

        self.optimizer.update(l=lb, u=ub)
        print("UPDATE TIME = ", time.time() - start)

        self.optimizer.update(l=lb, u=ub)

        start = time.time()

        res = self.optimizer.solve()
        print("SOLUTION TIME  ", self.nx,  time.time() - start)
        if (res.info.status_val == 1 or res.info.status_val == 2):
            pass
        else:
            print("NO ACCURATE SOLUTION FOUND")

        return res

    def solve_N(self, x0, N):
        Ax0 = np.matmul(self.A, np.array([x0]).T)

        nz = N*(self.nx + self.nu)
        neq = N*(self.nx + self.ny) + self.nt

        start = time.time()
        optimizer = osqp.OSQP()

        var_idx_cost = [i for i in range(0, nz-self.nx)] + \
            [self.nz - self.nx + i for i in range(self.nx)]
        var_idx = [i for i in range(0, nz)]

        ineq_idx = [i for i in range(0, N*(self.nx))] + \
            [self.ne + i for i in range(0, N*self.ny+self.nt)]

        cost = self.cost[:, var_idx_cost][var_idx_cost, :]
        ineq = self.ineq_A[:, var_idx][ineq_idx, :]
        lb = self.ineq_l[ineq_idx, :]
        ub = self.ineq_u[ineq_idx, :]

        if len(self.F) <= N:
            F = self.F[-1]
        else:
            F = self.F[N]

        ineq[-self.nt:, -self.nx:] = F.P
        lb[-self.nt:, :] = F.lb
        ub[-self.nt:, :] = F.ub

        lb[0: self.nx] = Ax0
        ub[0: self.nx] = Ax0

        optimizer.setup(P=cost, q=np.zeros((nz, 1)),
                        A=ineq, l=lb, u=ub, **self.settings)
        # if self.x0 is not None:
        #    if self.N_old >= N:
        #        x0 = self.x0[0:nz]
        #        optimizer.warm_start(x=x0)
        #    else:
        #        x0 = np.pad(self.x0, (0, (N-self.N_old)
        #                              * (self.nx+self.nu)))
        #        optimizer.warm_start(x=x0)

        res = optimizer.solve()

        # if res.info.status_val == 1:
        #    for i in range(1, 3):
        #        xu = res.x[(self.nx + self.nu)*(N-i) +
        #                   self.nu:(self.nx + self.nu)*(N-i+1)].reshape((self.nx, 1))
        #        hej = self.F[0].A @ xu
        #        print(sum(hej <= self.F[0].b) == self.nt*2)

        # print("result")
        #print(sum(np.matmul(self.C2, np.array([xu[self.nu:]]).T).T))

        # self.x0 = res.x
        # self.y0 = res.y
        # self.N_old = N

        return res

    def initial_solve(self, x0):
        self.solve(x0, 1.5)

    def solve(self, x0, time_limit=0.06, dN=1):
        init_time = time.time()
        N = max(self.N0-1, 5)
        res = self.solve_N(x0, N)

        if (res.info.status_val == 1):
            if N > 5:
                cmin = res.info.obj_val + N
                print("* Attempt to decrease horizon from %i, value = %f" %
                      (N, cmin))
                # If feasible, try smaller
                while time.time() - init_time < time_limit and N > 5:
                    N -= dN
                    res = self.solve_N(x0, N)
                    if (res.info.status_val == 1):
                        if res.info.obj_val + N < cmin:
                            print("* FOUND LOWER VALUE: %f < %f" %
                                  (res.info.obj_val, cmin))
                            cmin = res.info.obj_val + N

                        else:
                            print("* Decrease to %i does not lower cost: %f > %f"
                                  % (N, res.info.obj_val+N, cmin))
                            N += 1
                            break
                    else:
                        print("* Decrease not feasible")
                        N += 1
                        break
                print("* Recompute with N = %i" % N)
                res = self.solve_N(x0, N)

        else:
            print("* Have to increase horizon")
            feasible = False
            while not feasible and N <= self.N and time.time() - init_time < time_limit:
                print("Solve with N = ", N)
                N += 1
                res = self.solve_N(x0, N)
                if (res.info.status_val == 1):
                    feasible = True

        self.N0 = N
        return res.info.status_val, res.x[-N*(self.nx+self.nu):], N


if __name__ == "__main__":

    print("START")
    np.set_printoptions(precision=5, linewidth=180,
                        edgeitems=8, suppress=True)
    Ts = 0.05
    N = 50
    x0 = np.array([[1, 4, 0.5, 0.1, 0, 0]]).T

    # Define constraints
    v_max = 8.0
    a_max = 5
    u_max = 5
    h_max = 100
    h_min = -0.1
    w_max = 0.25*Ts*u_max
    tau_a = 0.9
    h_s = 2
    d_s = 2.3
    d_l = 0.8
    v_td = 0.8

    # States: x, h, vx, vh, ax, ah
    A = np.array([[0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 1],
                  [0, 0, 0, 0, -2, 0],
                  [0, 0, 0, 0, 0, -2]])
    B = np.array([[0, 0],
                  [0, 0],
                  [0, 0],
                  [0, 0],
                  [2, 0],
                  [0, 2]])
    # Controlled variables: x, h, ux, uh
    C = np.array([[1, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0]])
    D = np.array([[0, 0],
                  [0, 0],
                  [1, 0],
                  [0, 1]])

    nx = A.shape[0]
    nu = B.shape[1]
    ny = C.shape[0]
    dim = int(nx/3)
    hrz_idx = np.arange(0, nx, dim)
    vrt_idx = np.arange(1, nx, dim)

    # Phi = np.eye(nx)*Ts +
    #    (np.linalg.matrix_power(A, 2)*Ts**2)/2 +
    #    (np.linalg.matrix_power(A, 2)*Ts**3)/6 +
    #    (np.linalg.matrix_power(A, 3)*Ts**4)/24
    #A = np.eye(nx) + np.matmul(A, Phi)
    #B = np.matmul(Phi, B)

    A_hrz = A[hrz_idx, :][:, hrz_idx]
    B_hrz = B[hrz_idx, None,  0]
    C_hrz = C[[0, 2], :][:, hrz_idx]
    D_hrz = D[[0, 2], None, 0]

    A_vrt = A[vrt_idx, :][:, vrt_idx]
    B_vrt = B[vrt_idx, None, 1]
    C_vrt = C[[1, 3], :][:, vrt_idx]
    D_vrt = D[[1, 3], None, 1]

    nvx = A_vrt.shape[0]
    nvu = B_vrt.shape[1]
    nvy = C_vrt.shape[0]
    nhx = A_hrz.shape[0]
    nhu = B_hrz.shape[1]

    # Weights
    Q_hrz = np.array([[5, 0, 0],
                      [0, 4, 0],
                      [0, 0, 3]])
    R_hrz = np.eye(nhu)*1
    Qf_hrz = np.array([[5, 0, 0],
                       [0, 1, 0],
                       [0, 0, 1]])

    Q_vrt = np.array([[1, 0, 0],
                      [0, 2, 0],
                      [0, 0, 1]])

    R_vrt = np.eye(nvu)*10
    Qf_vrt = np.eye(nvx)*5

    # Terminal constraint Q: Q[0]*x <= Q[1]
    # F = [np.zeros((8, nx)), np.zeros((8, 1))]
    # F[0][np.arange(0, 8, 2), 0: 4] = np.eye(4)
    # F[0][np.arange(1, 8, 2), 0: 4] = -np.eye(4)
    # F[1] = np.array([[d_l, ], [0.05], [d_l], [-h_min], *[[v_td]]*4])

    F_hrz = [np.zeros((4, nhx)), np.zeros((4, 1))]
    F_hrz[0][0: 4, 0: 2] = [[1, 0], [-1, 0], [0, 1], [0, -1]]
    F_hrz[1] = np.array([[d_l], [d_l], *[[v_td]]*2])

    F_vrt = [np.zeros((4, nvx)), np.zeros((4, 1))]
    F_vrt[0][0: 4, 0: 2] = [[1, 0], [-1, 0], [0, 1], [0, -1]]
    F_vrt[1] = np.array([[0.05], [-h_min], *[[v_td]]*2])

    # State/input constraint
    Y = [np.zeros((4, ny)), np.zeros((4, 1)), np.zeros((4, 1))]
    Y[0][0: 4, :] = np.eye(ny)
    Y[1] = np.array([[-100], [1], *[[-u_max]]*2])  # lower limit
    Y[2] = np.array([[100], [100], *[[u_max]]*2])  # upper limit

    Y_hrz = [Y[0][[0, 2], :][:, [0, 2]], Y[1][[0, 2]], Y[2][[0, 2]]]
    Y_vrt = [Y[0][[1, 3], :][:, [1, 3]], Y[1][[1, 3]], Y[2][[1, 3]]]

    # Disturbance set
    W = [np.array([[1.0,  0.0,   0.0],
                   [0.0,  1.0,   0.0],
                   [0.0,  0.0,   1.0],
                   [-1.0,  0.0,   0.0],
                   [0.0, -1.0,   0.0],
                   [0.0,  0.0,  -1.0]]), np.zeros((6, 1))]
    W[1] = np.array([[*[0.0, 0.0, 0.01]*2]]).T

    Fset = Polyhedron(F_hrz[0], b=F_hrz[1], name='Terminal set')
    Yset = Polyhedron(Y_hrz[0], lb=Y_hrz[1],
                      ub=Y_hrz[2], name='Constraint set')
    W = Polyhedron(W[0], b=W[1], name='Disturbance set')

    Yall, Qall = reachability_matrices(
        A_hrz, B_hrz, C_hrz, D_hrz, W, Fset, Yset, 12, 50)

    ctrl = ControllerOSQP(A_hrz, B_hrz, C_hrz, D_hrz, N, Ts, 2)

    ctrl.set_cost_matrix(Q_hrz, R_hrz, Qf_hrz)
    ctrl.set_inequality_constraints(Yall)
    ctrl.set_terminal_constraint(Qall)
    ctrl.setup_problems()

    Fset = Polyhedron(F_vrt[0], b=F_vrt[1], name='Terminal set')
    Yset = Polyhedron(Y_vrt[0], lb=Y_vrt[1],
                      ub=Y_vrt[2], name='Constraint set')

    ctrl_vrt = ControllerOSQP(A_vrt, B_vrt, C_vrt, D_vrt, N, Ts, 2)
    ctrl_vrt.set_cost_matrix(Q_vrt, R_vrt, Qf_vrt)

    Yall, Qall = reachability_matrices(
        A_vrt, B_vrt, C_vrt, D_vrt, W, Fset, Yset, 12, 50)

    ctrl_vrt.set_inequality_constraints(Yall)
    ctrl_vrt.set_terminal_constraint(Qall)
    ctrl_vrt.setup_problems()

    res = ctrl.solve(x0[[0, 2, 4], :])
    u = res.x[np.arange(0, 4*N, 4)]
    x = res.x[np.arange(1, 4*N, 4)]
    v = res.x[np.arange(2, 4*N, 4)]
    a = res.x[np.arange(3, 4*N, 4)]

    scaling = np.ones((ctrl_vrt.ni, 1))
    scaling[np.arange(1, ctrl_vrt.ni, ctrl_vrt.ny), 0] = np.maximum(
        np.minimum((h_s * np.abs(x) - h_s*d_l)/(d_s-d_l), h_s), -0.01)

    lower_bound = ctrl_vrt.ineq_l.copy()

    lower_bound[ctrl_vrt.ne: ctrl_vrt.ni+ctrl_vrt.ne] *= scaling
    res = ctrl_vrt.solve(x0[[1, 3, 5], :], lower_bound)

    u = res.x[np.arange(0, 4*N, 4)]
    h = res.x[np.arange(1, 4*N, 4)]
    vh = res.x[np.arange(2, 4*N, 4)]
    ah = res.x[np.arange(3, 4*N, 4)]

    # PLOT THE RESULTS ---------------------------------
    plt.figure()
    plt.subplot(3, 2, 1)
    plt.plot(np.arange(0, Ts*N, Ts), x)
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.subplot(3, 2, 3)
    plt.plot(np.arange(0, Ts*N, Ts), v)
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.subplot(3, 2, 5)
    plt.plot(np.arange(0, Ts*N, Ts), a)
    plt.xlabel('Time [s]')
    plt.ylabel('Acceleration [m/s2]')

    plt.subplot(3, 2, 2)
    plt.plot(np.arange(0, Ts*N, Ts), h)
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.subplot(3, 2, 4)
    plt.plot(np.arange(0, Ts*N, Ts), vh)
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.subplot(3, 2, 6)
    plt.plot(np.arange(0, Ts*N, Ts), ah)
    plt.xlabel('Time [s]')
    plt.ylabel('Acceleration [m/s2]')

    plt.suptitle('Test MPC')

    plt.figure()
    plt.plot(x, h)
    plt.xlabel('Distance [m]')
    plt.ylabel('Altitude [m]')

    plt.plot([d_l, d_s, 7], [0, h_s, h_s], 'r')
    plt.plot([-d_l, -d_s, -7], [0, h_s, h_s], 'r')

    plt.show()
