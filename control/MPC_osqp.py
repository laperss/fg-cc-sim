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
import math
import sys


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

        self.cmin_last = 0
        self.cmin_pred = 0
        self.xpred = np.zeros((self.nx, 1))

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
                         'time_limit': 5.5, 'eps_abs': 1e-5, 'eps_rel': 1e-5, 'max_iter': 30000}

        self.optimizer = osqp.OSQP()

    def set_cost_matrix(self, Q, R, Qf, F=None, G=None, q=None, r=None, qf=None):

        print("* UPDATE THE COST MATRIX")
        print("Q = ")
        print(Q)
        print(q)
        if F is not None:
            Q = np.matmul(F.T, np.matmul(Q, F))
            Qf = np.matmul(F.T, np.matmul(Qf, F))

        if G is not None:
            R = np.matmul(G.T, np.matmul(R, G))

        if q is None:
            print("No linear state cost given")
            q = np.zeros((self.nx, 1))
        if qf is None:
            qf = q
        if r is None:
            print("No linear input cost given")
            r = np.zeros((self.nu, 1))

        self.Q0 = Q
        self.R0 = R
        cost = block_diag([R, scipy.sparse.kron(scipy.sparse.eye(self.N-1),
                                                block_diag([Q, R])), Qf], format='csc')

        print(Q)

        print(q)

        print(R)
        print(r)
        cost_linear = np.vstack((r,
                                 (np.tile(np.concatenate((q, r), axis=0), (self.N-1, 1))),
                                 qf))

        self.cost = cost
        self.cost_linear = cost_linear

    def add_cross_terms(self, i, j):
        cost = self.cost.todense()

        print(cost.shape)
        for itr in range(1, self.N):
            cost[itr*(self.nx+self.nu)+i, (itr-1)*(self.nx+self.nu)+i]
            cost[(itr-1)*(self.nx+self.nu)+i, itr*(self.nx+self.nu)+i]

        return

    def set_equality_constraints(self, A, B):
        ineq_A = np.zeros((self.ne, self.nz))
        ineq_l = np.zeros((self.ne, 1))

        # Uncommented for including known u0:
        # ineq_A[0: self.nx, 0: self.nu] = -B
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
        self.optimizer.setup(P=self.cost, q=self.cost_linear,
                             A=self.ineq_A, l=self.ineq_l, u=self.ineq_u, **self.settings)
        res = self.optimizer.solve()

    def initial_solve(self, x0, u0=None, lb=None, time_limit=0.1):
        status, path, N, c = self.solve(x0, u0=u0, lb=lb)
        return status, path, N, c

    def solve(self, x0, lb=None, u0=[0, 0, 0, 0], distance=None, time_limit=0.1, k=0):

        # x0 = np.array([x0]).T
        if u0 is None:
            Ax0 = np.matmul(self.A, x0)
        else:
            Ax0 = np.matmul(self.A, x0) + np.matmul(self.B, u0)

        # print("x0 = ", Ax0.T)

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
        if (res.info.status_val == 1):
            pass
        else:
            print("NO ACCURATE SOLUTION FOUND: ", res.info.status_val)

        self.x0 = res.x

        return res.info.status_val, res.x, self.N, res.info.obj_val


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

        Q = [None for i in range(p)]  # from 0 to P: then constant
        Y = [None for i in range(p)]  # from 0 to P: then constant
        L = [None for i in range(p)]  # from 0 to P: then zero

        L[0] = np.eye(self.nx)
        Q[0] = Xf
        Y[0] = Y0

        # The disturbance will go to zero in P steps.
        # There are P+1 different K matrices
        for i in range(p-1):
            print("Y[%i] = " % i)
            print(Y[i].lb.T)
            print(Y[i].ub.T)

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
        print("* (NEW) Number of terminal constraints = %i" % self.nt)
        self.F = F

        if type(F) != list:
            raise TypeError("F must be a list of sets.")

        if len(F) < self.N:
            F = F[-1]
        else:
            F = F[self.N]

        super().set_terminal_constraint(F)

    def set_terminal_constraint_big_M(self, F):
        """ Add array of all terminal constraints """
        print("* (BIG M) Number of terminal constraints = %i" % self.nt)
        self.F = F

        ineq_A = np.zeros((self.nt*self.N, self.nz))
        ineq_l = np.zeros((self.nt*self.N, 1))
        ineq_u = np.zeros((self.nt*self.N, 1))

        for i in range(self.N):
            if i < len(F):
                ineq_A[self.nt*i: self.nt*(i+1), (i+1)*self.nu + i *
                       self.nx: (i+1)*(self.nu+self.nx)] = F[i].P.todense()
                ineq_l[self.nt*i: self.nt*(i+1)] = F[i].lb  # Lower bound
                ineq_u[self.nt*i: self.nt*(i+1)] = F[i].ub  # Upper bound

            else:
                ineq_A[self.nt*i: self.nt*(i+1),
                       (i+1)*self.nu + i*self.nx: (i+1)*(self.nu+self.nx)] = F[-1].P.todense()
                ineq_l[self.nt*i: self.nt*(i+1)] = F[-1].lb  # Lower bound
                ineq_u[self.nt*i: self.nt*(i+1)] = F[-1].ub  # Upper bound

        print(ineq_A)

        self.ineq_A[self.ne+self.ni:, :] = ineq_A
        self.ineq_l[self.ne+self.ni:, :] = ineq_l
        self.ineq_u[self.ne+self.ni:, :] = ineq_u


class ControllerOSQPRobustVariableHorizon(ControllerOSQPRobust):
    def __init__(self, A, B, C, D, N, ds, nt=0):
        super().__init__(A, B, C, D, N, ds, nt)
        print("N = ", self.N)
        self.N0 = int(self.N/3)
        print("N = ", self.N)
        print("N0 = ", self.N0)
        self.big_M = False

        if self.big_M:
            self.nc = self.ni + self.ne + self.nt*self.N  # total number of constraints
            self.ineq_A = np.zeros((self.nc, self.nz))
            self.ineq_l = np.zeros((self.nc, 1))
            self.ineq_u = np.zeros((self.nc, 1))

            self.set_equality_constraints(A, B)

    def set_terminal_constraint(self, F):
        if self.big_M:
            self.set_terminal_constraint_big_M(F)
        else:
            super().set_terminal_constraint(F)

    def solve_big_M(self, x0, u0, N, lb):
        # print("\nSOLVE USING BIG M METHOD: %i" % self.nx)
        # print("SOLVE N = ", N)
        Ax0 = np.matmul(self.A, x0) + np.matmul(self.B, u0)

        # print("x1 = \n", Ax0)

        start = time.time()
        if lb is None:
            lb = self.ineq_l.copy()
        else:
            lb = lb.copy()

        ub = self.ineq_u.copy()

        # Equalities: 0:N*nx
        # Inequalities: Nmax*nx:Nmax*nx + N*ny

        # Inequality: remove last (Nmax-N) with big M
        lb[self.ne+N*self.ny:self.ne+self.N*self.ny,
            :] -= np.ones(((self.N-N)*self.ny, 1))*100
        ub[self.ne+N*self.ny:self.ne+self.N*self.ny,
            :] += np.ones(((self.N-N)*self.ny, 1))*100

        # Terminal: remove first N with big M
        lb[self.ne+self.ni:self.ne+self.ni+(N-1)*self.nt,
            :] -= np.ones(((N-1)*self.nt, 1))*100
        ub[self.ne+self.ni:self.ne+self.ni+(N-1)*self.nt,
            :] += np.ones(((N-1)*self.nt, 1))*100

        # Terminal: remove last with big M
        lb[self.ne+self.ni+N*self.nt:self.ne+self.ni + self.N*self.nt,
           :] -= np.ones(((self.N-N)*self.nt, 1))*100
        ub[self.ne+self.ni+N*self.nt:self.ne+self.ni + self.N*self.nt,
            :] += np.ones(((self.N-N)*self.nt, 1))*100

        # lb[(self.N-N)*self.nx: (self.N-N)*self.nx + self.nx] = Ax0
        # ub[(self.N-N)*self.nx: (self.N-N)*self.nx + self.nx] = Ax0
        lb[0:self.nx] = Ax0
        ub[0:self.nx] = Ax0

        # print("EQUALITY =")
        # print(np.concatenate(
        #    (lb[0:self.ne], ub[0:self.ne]), axis=1))

        # print("CONSTRAINTS =")
        # print(np.concatenate(
        #    (lb[self.ne:self.ne+self.ni], ub[self.ne:self.ne+self.ni]), axis=1))

        # print("TERMINAL =")
        # print(np.concatenate(
        # x   (lb[self.ne+self.ni:], ub[self.ne+self.ni:]), axis=1))

        self.optimizer.update(l=lb, u=ub)
        # print("UPDATE TIME = ", time.time() - start)

        self.optimizer.update(l=lb, u=ub)

        start = time.time()

        res = self.optimizer.solve()
        # print("SOLUTION TIME  ", self.nx,  time.time() - start)
        if (res.info.status_val == 1 or res.info.status_val == 2):
            pass
        else:
            print("NO ACCURATE SOLUTION FOUND: ", res.info.status_val)

        # print("--------------------------")

        return res

    def solve_new(self, x0, u0, N, lb):
        if lb is None:
            lb = self.ineq_l.copy()

        Ax0 = np.matmul(self.A, x0) + np.matmul(self.B, u0)

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
        lb = lb[ineq_idx, :]
        ub = self.ineq_u[ineq_idx, :]

        if len(self.F) <= N:
            F = self.F[-1]
        else:
            F = self.F[N]

        ineq[-self.nt:, -self.nx:] = F.P
        lb[-self.nt:, :] = F.lb
        ub[-self.nt:, :] = F.ub

        lb[0: self.nx, :] = Ax0
        ub[0: self.nx, :] = Ax0

        # print("CONSTRAINTS =")
        # print(np.concatenate(
        #    (lb[N*(self.nx):N*(self.nx+self.ny)], ub[N*(self.nx):N*(self.nx+self.ny)]), axis=1))
        # print("TERMINAL =")
        # print(np.concatenate(
        #    (lb[N*(self.nx+self.ny):], ub[N*(self.nx+self.ny):]), axis=1))

        # print(self.ineq_A.todense()[-10:, -30:])

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

        # self.x0 = res.x
        # self.y0 = res.y
        # self.N_old = N

        return res

    def solve_N(self, x0, u0, N, lb):
        if self.big_M:
            res = self.solve_big_M(x0, u0, N, lb)
        else:
            res = self.solve_new(x0, u0, N, lb)

        return res

    def initial_solve(self, x0, u0, lb=None, time_limit=1.8):
        print(x0)
        print(u0)
        status, path, N, c = self.solve(x0, u0, time_limit, lb=lb)
        print("**********************************\n********************************\n\nINITIAL HORIZON (%i): %i" % (self.nx, N))
        return status, path, N, c

    def solve(self, x0, u0, time_limit=0.05, dN=1, lb=None):
        init_time = time.time()
        N0 = self.N0  # Old value

        Nmin = 30

        N1 = max(self.N0-1, Nmin)
        res1 = self.solve_N(x0, u0, N1, lb)

        if (res1.info.status_val != 1):
            print("*** WARNING: DOES NOT FULFILL THE CONSTRAINTS ***")
            print("* Have to increase horizon")
            feasible = False
            while not feasible and N1 <= self.N and time.time() - init_time < time_limit:
                print("Solve with N = ", N1)
                N1 += 2
                res1 = self.solve_N(x0, u0, N1, lb)
                if (res1.info.status_val == 1):
                    feasible = True
            if not feasible:
                print("ERROR: COULD NOT FIND A FEASIBLE SOLUTION")
                print("Time limit = ", time.time() - init_time, time_limit)
                print("Latest N = ", N1)
                print("x0 = ", x0.T)

        else:
            feasible = True
            if N1 > Nmin:
                cmin = res1.info.obj_val + N1

                # Decrease in accordance with predicted path
                # print("n = %i:   N = %i,  c = %f <= %f"
                #      % (self.nx, N1, cmin, self.cmin_pred))
                # print(x0)
                # print(self.xpred.T)
                # print(self.xpred.T - x0)

                if cmin <= self.cmin_pred:
                    pass
                # print("n = %i:   N = %i,  c = %f <= %f"
                 #         % (self.nx, N1, cmin, self.cmin_pred))

                else:
                    # Try with previous horizon
                    Nnew = N0
                    res_new = self.solve_N(x0, u0, Nnew, lb)
                    cnew = res_new.info.obj_val + Nnew

                    #path_new = res_new.x
                    # u0_ = path_new[np.arange(
                    #    0, Nnew*(self.nx+self.nu), (self.nx+self.nu))]
                    # u1_ = path_new[np.arange(
                    #    1, Nnew*(self.nx+self.nu), (self.nx+self.nu))]
                    # u2_ = path_new[np.arange(
                    #    2, Nnew*(self.nx+self.nu), (self.nx+self.nu))]
                    # u3_ = path_new[np.arange(
                    #    3, Nnew*(self.nx+self.nu), (self.nx+self.nu))]
                    # print("------------------------------------------")
                    # print(u0_)
                    # print(u1_)
                    # print(u2_)
                    # print(u3_)
                    # Must increase horizon
                    if cnew <= cmin:
                        print("* Minimum found by increase")
                        while time.time() - init_time < time_limit and cnew <= cmin:
                            if cnew <= self.cmin_pred:
                                break
                            else:
                                # Save these values, optimal for now
                                cmin = cnew
                                N1 = Nnew
                                res1 = res_new
                                print("\t* New optimal N: ", N1, cmin)

                                Nnew += 1
                                res_new = self.solve_N(x0, u0, Nnew, lb)
                                cnew = res_new.info.obj_val + Nnew

                                #path_new = res_new.x

                                # u0_ = path_new[np.arange(
                                #    0, Nnew*(self.nx+self.nu), (self.nx+self.nu))]
                                # u1_ = path_new[np.arange(
                                #    1, Nnew*(self.nx+self.nu), (self.nx+self.nu))]
                                # u2_ = path_new[np.arange(
                                #    2, Nnew*(self.nx+self.nu), (self.nx+self.nu))]
                                # u3_ = path_new[np.arange(
                                #    3, Nnew*(self.nx+self.nu), (self.nx+self.nu))]
                                # print(
                                #    "-------------NEW N = %i: c = %f---------------------" % (Nnew, cnew))
                                # print(u0_)
                                # print(u1_)
                                # print(u2_)
                                # print(u3_)

                    else:
                        # print("Minimum found by decrease")
                        Nnew = N0 - 2
                        res_new = self.solve_N(x0, u0, Nnew, lb)
                        cnew = res_new.info.obj_val + Nnew

                        if self.cmin_pred == 0:
                            while time.time() - init_time < time_limit and cnew <= cmin and N1 >= Nmin:
                                # print("\t N = %i, cmin = %f" % (N1, cmin))
                                if (res1.info.status_val == 1):
                                    if cnew <= cmin:
                                        # Optimal for now
                                        res1 = res_new
                                        cmin = cnew
                                        N1 = Nnew

                                        Nnew -= 1
                                        res_new = self.solve_N(
                                            x0, u0, Nnew, lb)
                                        cnew = res_new.info.obj_val + Nnew

                                    else:
                                        N1 = N1 + 1
                                        # print("N = %i minimum found" % N1)
                                        break
                                else:
                                    print("N = %i not feasible" % N1)
                                    N1 = N1+1
                                    break

                        else:
                            while time.time() - init_time < time_limit and cnew < cmin:
                                if (res1.info.status_val == 1):
                                    if cnew < self.cmin_pred:
                                        break
                                    else:

                                        res1 = res_new
                                        cmin = cnew
                                        N1 = Nnew

                                        Nnew -= 1
                                        res_new = self.solve_N(
                                            x0, u0, Nnew, lb)
                                        cnew = res_new.info.obj_val + Nnew

                                else:
                                    break

                # If feasible, try smaller
                if False:
                    # print("*\tn = %i:   N = %i,  c = %f >= %f"
                    #      % (self.nx, N, cmin, self.cmin_pred))

                    while time.time() - init_time < time_limit and N1 > Nmin:
                        N1 -= 1
                        res1 = self.solve_N(x0, u0, N1, lb)
                        if (res1.info.status_val == 1):
                            if res1.info.obj_val + N < self.cmin_pred:
                                break

                        if (res1.info.status_val == 1):
                            if res1.info.obj_val + N < cmin:
                                # print("* FOUND LOER VALUE: %f < %f" %
                                #      (res1.info.obj_val, cmin))
                                cmin = res1.info.obj_val + N

                            else:
                                # print("* Decrease to %i does not lower cost: %f > %f"
                                #      % (N1, res1.info.obj_val+N, cmin))
                                N1 += 1
                                break
                        else:
                            # print("* Decrease not feasible")
                            N += 1
                            break
                    # print("* Recompute with N = %i" % N)
                    res1 = self.solve_N(x0, u0, N1, lb)

        self.N0 = N1
        self.cmin_last = res1.info.obj_val + N1

        # If recreating
        if self.big_M:
            path = res1.x  # [0:N1*(self.nx+self.nu)]
        else:
            path = res1.x
        # If big M method: Return initial part only
        if feasible:
            x1 = np.array([path[self.nu:self.nu+self.nx]]).T
            u0 = np.array([path[0:self.nu]]).T
            self.xpred = x1

            self.cmin_pred = (self.cmin_last - 1 -
                              np.matmul(np.matmul(x1.T, self.Q0), x1) -
                              np.matmul(np.matmul(u0.T, self.R0), u0))

        return res1.info.status_val, path, N1, self.cmin_last


if __name__ == "__main__":

    from dynamical_models import get_horizontal_dynamics, get_vertical_dynamics
    from mpc_sets import get_mpc_sets
    from LQR import get_lqr_feedback

    import random
    print("START")
    np.set_printoptions(precision=5, linewidth=180,
                        edgeitems=8, suppress=True)

    Nmax = 100
    v_ref = 20
    Ts = 0.12
    h_s = 2
    d_s = 2.3
    d_l = 0.5

    A_c_vrt, B_c_vrt, C_vrt, D_vrt = get_vertical_dynamics()
    A_c, B_c, C_hrz, D_hrz, H_hrz, G_hrz, B_dc, H_hrz_new = get_horizontal_dynamics(
        v_ref)

    nx = A_c.shape[0]
    nu = B_c.shape[1]

    nvar = nx + nu

    ny = C_hrz.shape[0]

    Phi = np.eye(nx)*Ts + \
        (np.linalg.matrix_power(A_c, 1)*Ts**2)/2 + \
        (np.linalg.matrix_power(A_c, 2)*Ts**3)/6 + \
        (np.linalg.matrix_power(A_c, 3)*Ts**4)/24
    A_hrz = np.eye(nx) + np.matmul(A_c, Phi)
    B_hrz = np.matmul(Phi, B_c)
    Bd_hrz = np.matmul(Phi, B_dc)

    Phi = np.eye(A_c_vrt.shape[0])*Ts + \
        (np.linalg.matrix_power(A_c_vrt, 1)*Ts**2)/2 + \
        (np.linalg.matrix_power(A_c_vrt, 2)*Ts**3)/6 + \
        (np.linalg.matrix_power(A_c_vrt, 3)*Ts**4)/24
    A_vrt = np.eye(A_c_vrt.shape[0]) + np.matmul(A_c_vrt, Phi)
    B_vrt = np.matmul(Phi, B_c_vrt)

    Q_vrt, R_vrt, Fset_vrt, Yset_vrt, W_vrt, Q_hrz, R_hrz, Fset, Yset, W, q_hrz, r_hrz, Q_hrz_new, q_hrz_new = get_mpc_sets(
        A_hrz, B_hrz, Bd_hrz, A_vrt, B_vrt)

    mpc_hrz = ControllerOSQPRobustVariableHorizon(
        A_hrz, B_hrz, C_hrz, D_hrz, Nmax, Ts, 5)

    mpc_vrt = ControllerOSQPRobustVariableHorizon(
        A_vrt, B_vrt, C_vrt, D_vrt, Nmax, Ts, 2)

    Y, Q = mpc_hrz.reachability_matrices(W, Fset, Yset, 25)

    if False:
        Q_hrz = Q_hrz[[0, 1, 2, 5, 6, 7, 8, 9], :][:, [0, 1, 2, 5, 6, 7, 8, 9]]
        H_hrz = H_hrz[[0, 1, 2, 5, 6, 7, 8, 9], :]
        q_hrz = q_hrz[[0, 1, 2, 5, 6, 7, 8, 9], :]

    print("-----------------")
    print(Q_hrz)
    print(H_hrz)
    Q2 = np.matmul(H_hrz.T, np.matmul(Q_hrz, H_hrz))
    R2 = np.matmul(G_hrz.T, np.matmul(R_hrz, G_hrz))
    print("-----------------")
    print(Q2)
    print(R2)

    q2 = np.matmul(q_hrz.T, H_hrz).T
    r2 = np.matmul(r_hrz.T, G_hrz).T
    print("-----------------")
    print(q2)
    print(r2)

    LQRgain, Qf, qf = get_lqr_feedback(A_hrz, B_hrz, Q2, R2, q2)

    Qf = Qf

    print("Teminal = ")
    print(Qf)
    mpc_hrz.set_cost_matrix(Q2, R2, Qf, q=q2, r=r2)
    mpc_hrz.set_inequality_constraints(Y)
    mpc_hrz.set_terminal_constraint(Q)
    mpc_hrz.setup_problems()

    Y, Q = mpc_vrt.reachability_matrices(
        W_vrt, Fset_vrt, Yset_vrt, 20)

    Qf = 5*Q_vrt
    mpc_vrt.set_cost_matrix(Q_vrt, R_vrt, Qf)
    mpc_vrt.set_inequality_constraints(Y)
    mpc_vrt.set_terminal_constraint(Q)
    mpc_vrt.setup_problems()

    state_t0 = np.array(
        [[98.0, 5.0, 20, 0.0, 0.0,  0.0, 102,  0,  22, 0.0, 0]]).T
    input_t = np.array([[0, 0, 0, 0]]).T

    status, path, N, c = mpc_hrz.initial_solve(
        state_t0, input_t, time_limit=25.0)

    solver_time = []
    N_iters = []
    if mpc_hrz.big_M:
        Adense = mpc_hrz.ineq_A.copy().todense()
    else:
        Adense = mpc_hrz.ineq_A.copy().todense()

    for i in range(500):
        state_t = state_t0 + np.array([[random.randrange(-1000, 1000)*0.001,
                                        random.randrange(-50, 50)*0.01,
                                        random.randrange(-50, 50)*0.01,
                                        random.randrange(-50, 50)*0.001,
                                        0.0,
                                        0.0,
                                        random.randrange(-1000, 1000)*0.001,
                                        random.randrange(-50, 50)*0.01,
                                        random.randrange(-100, 100)*0.01,
                                        random.randrange(-50, 50)*0.001,
                                        0.0]]).T

        # print(state_t.T)
        mpc_vrt.N0 = N + 10
        start = time.time()
        status, path, N, c = mpc_hrz.solve(state_t, input_t, time_limit=5.0)

        if mpc_hrz.big_M:
            #mpc_hrz.N0 = 32

            # Matrix with inequality constraints
            A = Adense[mpc_hrz.ne:mpc_hrz.ne+mpc_hrz.ny*N]
            # Vectors with lower/upper bounds
            lb = mpc_hrz.ineq_l[mpc_hrz.ne:mpc_hrz.ne+mpc_hrz.ny*N]
            ub = mpc_hrz.ineq_u[mpc_hrz.ne:mpc_hrz.ne+mpc_hrz.ny*N]
            if not (np.matmul(A, np.array([path]).T) >= lb - 0.01).all():
                print("LOWER constraint broken")
                print(np.concatenate((np.matmul(A, np.array([path]).T), lb,
                                      (np.matmul(A, np.array([path]).T) >= lb-0.01)), axis=1))

                x1 = path[np.arange(4, N*nvar, nvar)]
                y1 = path[np.arange(5, N*nvar, nvar)]
                v1 = path[np.arange(6, N*nvar, nvar)]
                a1 = path[np.arange(7, N*nvar, nvar)]
                psi1 = path[np.arange(8, N*nvar, nvar)]
                phi1 = path[np.arange(9, N*nvar, nvar)]
                x2 = path[np.arange(10, N*nvar, nvar)]
                y2 = path[np.arange(11, N*nvar, nvar)]
                v2 = path[np.arange(12, N*nvar, nvar)]
                a2 = path[np.arange(13, N*nvar, nvar)]
                psi2 = path[np.arange(14, N*nvar, nvar)]
                u0 = path[np.arange(0, N*nvar, nvar)]
                u1 = path[np.arange(1, N*nvar, nvar)]
                u2 = path[np.arange(2, N*nvar, nvar)]
                u3 = path[np.arange(3, N*nvar, nvar)]

                time = np.arange(0, N*Ts, Ts)
                plt.figure()
                plt.subplot(2, 2, 1)
                plt.plot(time, x1, label='UAV')
                plt.plot(time, x2, label='UGV')
                plt.title("Position x")
                plt.legend()

                plt.subplot(2, 2, 2)
                plt.plot(time, y1, label='UAV')
                plt.plot(time, y2, label='UGV')
                plt.title("Position y")
                plt.legend()

                plt.subplot(2, 2, 3)
                plt.plot(time, v1, label='UAV')
                plt.plot(time, v2, label='UGV')
                plt.plot(time[[0, -1]], [17, 17],
                         linestyle='dotted', color='c')
                plt.plot(time[[0, -1]], [25, 25],
                         linestyle='dotted', color='c')
                plt.title("Velocity")
                plt.legend()

                plt.subplot(2, 2, 4)
                plt.plot(time, psi1, label='UAV')
                plt.plot(time, psi2, label='UGV')
                plt.title("Heading")
                plt.legend()

                plt.figure()
                plt.subplot(2, 2, 1)
                plt.plot(time, a1, label='UAV')
                plt.plot(time, u0, linestyle='dashed', label='input')
                plt.plot(time[[0, -1]], [-1.5, -1.5],
                         linestyle='dotted', color='c')
                plt.plot(time[[0, -1]], [2.0, 2.0],
                         linestyle='dotted', color='c')
                plt.plot(time[[0, -1]], [-0.8, -0.8],
                         linestyle='dotted', color='b')
                plt.plot(time[[0, -1]], [0.8, 0.8],
                         linestyle='dotted', color='b')
                plt.title("Acceleration")
                plt.ylim([-1.7, 2.1])
                plt.legend()

                plt.subplot(2, 2, 2)
                plt.plot(time, a2, label='UGV')
                plt.plot(time, u2, linestyle='dashed', label='input')
                plt.plot(time[[0, -1]], [-3.8, -3.8],
                         linestyle='dotted', color='m')
                plt.plot(time[[0, -1]], [3.8, 3.8],
                         linestyle='dotted', color='m')
                plt.title("Acceleration")
                plt.ylim([-4.0, 4.0])
                plt.legend()

                plt.subplot(2, 2, 3)
                plt.plot(time, phi1, label='UAV')
                plt.plot(time, u2, linestyle='dashed', label='input')
                plt.plot(time[[0, -1]], [-0.2, -0.2],
                         linestyle='dotted', color='c')
                plt.plot(time[[0, -1]], [0.15, 0.15],
                         linestyle='dotted', color='c')
                plt.ylim([-0.25, 0.2])

                plt.title("Heading input")
                plt.legend()

                plt.subplot(2, 2, 4)
                plt.plot(time, psi2, label='UGV')
                plt.plot(time, u3, linestyle='dashed', label='input')
                plt.plot(time[[0, -1]], [-0.6, -0.6],
                         linestyle='dotted', color='m')
                plt.plot(time[[0, -1]], [0.6, 0.6],
                         linestyle='dotted', color='m')

                plt.ylim([-0.65, 0.65])

                plt.title("Heading input")
                plt.legend()

                plt.show()

                raise SystemExit

                if not (np.matmul(A, np.array([path]).T) <= ub + 0.01).all():
                    print("Upper constraint broken")

            if not (np.matmul(mpc_hrz.F[0].A,
                              np.array([path[(N-1)*mpc_hrz.nx + N*mpc_hrz.nu:N*(mpc_hrz.nx + mpc_hrz.nu)]]).T) <= mpc_hrz.F[0].b).all():
                print("TERMINAL CONSTRAINT BROKEN")
                print("Xf = ", path[-mpc_hrz.nx-1:])
                print(np.concatenate((np.matmul(mpc_hrz.F[0].A, np.array(
                    [path[(N-1)*mpc_hrz.nx + N*mpc_hrz.nu:N*(mpc_hrz.nx + mpc_hrz.nu)]]).T), mpc_hrz.F[0].b), axis=1))

                print(np.concatenate((np.matmul(mpc_hrz.F[0].A, np.array(
                    [path[-mpc_hrz.nx:]]).T), mpc_hrz.F[0].b), axis=1))

        else:
            A = Adense[mpc_hrz.ne:mpc_hrz.ne +
                       mpc_hrz.ny*N, 0:N*(mpc_hrz.nx+mpc_hrz.nu)]
            lb = mpc_hrz.ineq_l[mpc_hrz.ne:mpc_hrz.ne+mpc_hrz.ny*N]
            ub = mpc_hrz.ineq_u[mpc_hrz.ne:mpc_hrz.ne+mpc_hrz.ny*N]
            if not (np.matmul(A, np.array([path]).T) >= lb - 0.01).all():
                print("LOWER constraint broken")
                print(np.concatenate((np.matmul(A, np.array([path]).T), lb,
                                      (np.matmul(A, np.array([path]).T) >= lb - 0.01)), axis=1))

            if not (np.matmul(A, np.array([path]).T) <= ub + 0.01).all():
                print("Upper constraint broken")

            if not (np.matmul(mpc_hrz.F[0].A, np.array([path[-mpc_hrz.nx:]]).T) <= mpc_hrz.F[0].b).all():
                print("TERMINAL CONSTRAINT BROKEN")
                print("Xf = ", path[-mpc_hrz.nx:])

        solver_time.append(time.time() - start)
        N_iters.append(N)

        print("SOLVER TIME = ", N, solver_time[-1])

    fig, ax1 = plt.subplots()
    ax1.plot(solver_time, color='blue')
    ax1.set_ylabel('Solver time', color='blue')
    ax1.tick_params(axis='y', labelcolor='blue')
    plt.ylim([0, 1.0])
    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
    ax2.plot(N_iters, color='green')
    ax2.set_ylabel('Horizon', color='green')
    ax2.tick_params(axis='y', labelcolor='green')
    plt.ylim([-40, 60])

    plt.show()

    # N = Nmax
    # print("STATE = ", state_t)
    # print("STATE DIFF = ", state_t-predicted_state)
    x1 = path[np.arange(4, N*nvar, nvar)]
    y1 = path[np.arange(5, N*nvar, nvar)]
    v1 = path[np.arange(6, N*nvar, nvar)]
    a1 = path[np.arange(7, N*nvar, nvar)]
    psi1 = path[np.arange(8, N*nvar, nvar)]
    phi1 = path[np.arange(9, N*nvar, nvar)]
    x2 = path[np.arange(10, N*nvar, nvar)]
    y2 = path[np.arange(11, N*nvar, nvar)]
    v2 = path[np.arange(12, N*nvar, nvar)]
    a2 = path[np.arange(13, N*nvar, nvar)]
    psi2 = path[np.arange(14, N*nvar, nvar)]
    u0 = path[np.arange(0, N*nvar, nvar)]
    u1 = path[np.arange(1, N*nvar, nvar)]
    u2 = path[np.arange(2, N*nvar, nvar)]
    u3 = path[np.arange(3, N*nvar, nvar)]

    dist = np.array([math.sqrt((x1[i]-x2[i])**2 + (y1[i]-y2[i])**2)
                     for i in range(len(x1))])

    scale = np.ones((mpc_vrt.ni, 1))
    scale[np.arange(1, N*mpc_vrt.ny,
                    mpc_vrt.ny), 0] = np.maximum(
                        np.minimum((h_s * dist -
                                    h_s*d_l)/(d_s-d_l), h_s),
                        -0.05)

    scale[np.arange(N*mpc_vrt.ny+1, mpc_vrt.ni, mpc_vrt.ny),
          0] = np.ones(((Nmax-N)))*-0.05

    lower_bound = mpc_vrt.ineq_l.copy()
    lower_bound[mpc_vrt.ne: mpc_vrt.ne + mpc_vrt.ni] *= scale

    np.set_printoptions(precision=3, linewidth=200, threshold=10000,
                        edgeitems=105, suppress=True)

    status, path_vrt, N2, c2 = mpc_vrt.initial_solve(
        np.array([[10.3, 0.02]]).T, np.array([[0.0]]).T, time_limit=15, lb=lower_bound)

    status, path_vrt, N2, c2 = mpc_vrt.solve(
        np.array([[10.3, 0.02]]).T, np.array([[0.0]]).T, time_limit=15, lb=lower_bound)

    u0_vrt = path_vrt[0]
    if u0_vrt is None:
        print("VERTICAL ERROR", u0_vrt)
        print("x0 = ", 20.3, 0.02)
        print(x1)
        print(x2)
        print("deltax")
        print(x1-x2)
        print("v1 = ")
        print(path[np.arange(6, N*nvar, nvar)])
        print("v2 = ")
        print(path[np.arange(12, N*nvar, nvar)])
        print("DISTANCE")
        print(dist)
        u0_vrt = 0.0

    u5 = path_vrt[np.arange(0, 3*N2, 3)]
    h0 = path_vrt[np.arange(1, 3*N2, 3)]
    g0 = path_vrt[np.arange(2, 3*N2, 3)]

    # PLOT THE RESULTS ---------------------------------
    plt.figure()
    plt.subplot(3, 2, 1)
    plt.plot(np.arange(0, Ts*N, Ts), x1-x2, label='dx')
    plt.plot(np.arange(0, Ts*N, Ts), y1-y2, label='dy')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.legend()
    plt.subplot(3, 2, 3)
    plt.plot(np.arange(0, Ts*N, Ts), v1, label='v1')
    plt.plot(np.arange(0, Ts*N, Ts), v2, label='v2')
    plt.legend()
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.ylim([15, 25])
    plt.subplot(3, 2, 5)
    plt.plot(np.arange(0, Ts*N, Ts), a1, label='a1')
    plt.plot(np.arange(0, Ts*N, Ts), a2, label='v2')
    plt.legend()
    plt.xlabel('Time [s]')
    plt.ylabel('Acceleration [m/s2]')

    plt.subplot(3, 2, 2)
    plt.plot(np.linspace(0, Ts*N2, N2), h0)
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.subplot(3, 2, 4)
    plt.plot(np.arange(0, Ts*N, Ts), psi1, label='psi1')
    plt.plot(np.arange(0, Ts*N, Ts), psi2, label='psi2')
    plt.xlabel('Time [s]')
    plt.ylabel('Psi [rad]')
    plt.subplot(3, 2, 6)
    plt.plot(np.arange(0, Ts*N, Ts), phi1)
    plt.xlabel('Time [s]')
    plt.ylabel('Attitude rate [r]')

    plt.suptitle('Test MPC')

    plt.figure()
    if len(dist) >= len(h0):
        dist_ = np.concatenate(
            (np.array([dist]).T, np.ones((Nmax-N, 1))*dist[-1]), axis=0)
        h0_ = np.concatenate(
            (np.array([h0]).T, np.zeros((Nmax-N2, 1))), axis=0)
        plt.plot(dist_, h0_, color='green')
    else:
        plt.plot(dist, h0[0:len(dist)], color='green')
        plt.plot(np.ones((1+len(h0)-len(dist), 1)) *
                 dist[-1], h0[len(dist)-1:], color='red')

    plt.xlabel('Distance [m]')
    plt.ylabel('Altitude [m]')

    plt.plot([d_l, d_s, 7], [0, h_s, h_s], 'r')
    plt.plot([-d_l, -d_s, -7], [0, h_s, h_s], 'r')

    plt.show()
