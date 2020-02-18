import cdd
import numpy as np
from scipy.sparse import csc_matrix
import osqp
import utils


class Polyhedron:
    def __init__(self, A, b=None, Ae=None, be=None, lb=None, ub=None, name="Set"):
        self.has_bounds = False
        self.name = name

        # Determine what form is given in inputs
        if lb is not None and ub is not None:
            self.from_bounds = True
            self.has_bounds = True
            self.has_Ae = False
            self.add_minmax_bounds(A, lb, ub)
        elif b is not None:
            self.from_bounds = False
            self.has_Ab = True
            self.has_bounds = False
            self.add_Ab_form(A, b, Ae, be)

    def __str__(self):
        string = "\n%s = \n[" % self.name

        for row in range(self.ni):
            string += "["
            for column in range(self.dim):
                string += "   %8.4f" % self.A[row, column]
            if row < self.ni-1:
                string += "  ]\t\t[  %8.4f  ]" % self.b[row]
                string += "\n "
            else:
                string += "  ]] x <= \t[  %8.4f  ]" % self.b[row]
                string += "\n "

        return string

    def add_Ab_form(self, A, b, Ae, be):
        if b.shape[1] > 1:
            raise ValueError("Bounds must be column vectors")
        A[abs(A) < 1e-12] = 0.0
        b[abs(b) < 1e-12] = 0.0

        # print("b = ", b.T)
        self.dim = A.shape[1]
        self.ni = b.shape[0]
        self.A = A
        self.b = b

        if Ae is not None:
            self.ne = Ae.shape[0]
            self.Ae = Ae
            self.be = be
        else:
            self.ne = 0
            self.Ae = np.zeros((0, self.dim))
            self.be = np.zeros((0, 1))

        #self.add_cdd_polygon(A, b, Ae, be)

        self.Pul_from_Ab()

    def add_cdd_polygon(self, A, b, Ae, be):
        cdd_mat = cdd.Matrix(np.concatenate(
            (b, -A), axis=1).tolist(), number_type='float')

        if Ae is not None:
            cdd_mat.extend(np.concatenate(
                (be, -Ae), axis=1).tolist(), linear=True)

        try:
            self.cdd_poly = cdd.Polyhedron(cdd_mat)
        except RuntimeError:
            print("CDD Polyhedron could not be generated : %s" % self.name)
            print(np.concatenate((b, -A), axis=1))
            print(cdd_mat)
            raise RuntimeError

    def add_minmax_bounds(self, A, lb, ub):
        self.dim = A.shape[1]
        if lb is None or ub is None:
            raise ValueError(
                "You must set correct bounds on your Polyhedron")
        if lb.shape[1] > 1 or ub.shape[1] > 1:
            raise ValueError("Bounds must be column vectors")

        self.check_solution(A, lb, ub)

        if self.check_empty(lb, ub):
            self.P = csc_matrix(np.eye(self.dim))
            self.lb = np.zeros((self.dim, 1))
            self.ub = np.zeros((self.dim, 1))
        else:
            self.lb = lb
            self.ub = ub
            self.P = csc_matrix(A)

        self.has_bounds = True

        self.ni = lb.shape[0] + ub.shape[0]

        # Compute b from upper and lower bounds
        self.A = np.concatenate((A, -A))
        self.b = np.concatenate((ub, -lb))
        self.Ae = np.zeros((0, self.dim))
        self.be = np.zeros((0, 1))
        self.add_cdd_polygon(self.A, self.b, None, None)

    def check_solution(self, A, lb, ub):
        # Check that a feasible solution exists:
        settings = {'verbose': False, 'eps_abs': 1e-6,
                    'eps_rel': 1e-6, 'max_iter': 10000}
        m = osqp.OSQP()
        try:
            m.setup(P=csc_matrix(np.eye(self.dim)),
                    q=np.zeros((self.dim, 1)),
                    A=csc_matrix(A), l=lb, u=ub, **settings)
            results = m.solve()
        except:
            print("VALUE ERROR in %s. From bounds:  %i" %
                  (self.name, self.from_bounds))

            for i in range(len(lb)):
                if lb[i] > ub[i]:
                    print("Error in bound %i" % (i))
                    print("\t %f not less than %f" % (lb[i], ub[i]))

            print("LB = ", lb.T)
            print("UB = ", ub.T)
            print(self)
            print("--------------------")
            raise ValueError

    def check_empty(self, lb, ub):

        if min(lb) > -1e-6 and max(ub) < 1e-6:
            return True
        return False

    def Pul_from_Ab(self):
        """ Compute upper and lower bounds from b """
        # print("====================================")

        A = self.A.copy()
        b = self.b.copy()
        Ae = self.Ae.copy()
        be = self.be.copy()

        nrows = A.shape[0]
        ncols = A.shape[1]

        P = np.zeros((0, ncols))
        ub = np.zeros((0, 1))
        lb = np.zeros((0, 1))

        while nrows > 0:
            row0 = A[0, None, :]
            sign = np.sign(row0[0, np.where(row0 != 0)[1][0]])
            mirror = (A == -row0).all(axis=1).nonzero()[0]

            if len(mirror) > 0:
                if sign == 1:
                    ub = np.concatenate((ub, b[0, None, :]), axis=0)
                    lb = np.concatenate(
                        (lb, -b[mirror[0], None, :]), axis=0)
                    P = np.concatenate((P, row0), axis=0)
                elif sign == -1:
                    ub = np.concatenate(
                        (ub,  b[mirror[0], None, :]), axis=0)
                    lb = np.concatenate((lb, -b[0, None, :]), axis=0)
                    P = np.concatenate((P, -row0), axis=0)

                A = np.delete(A, mirror, 0)
                b = np.delete(b, mirror, 0)

            else:
                if sign == 1:
                    P = np.concatenate((P, row0), axis=0)
                    ub = np.concatenate((ub, b[0, None, :]), axis=0)
                    lb = np.concatenate((lb, np.array([[-np.inf]])), axis=0)
                elif sign == -1:
                    P = np.concatenate((P, -row0), axis=0)
                    ub = np.concatenate((ub, np.array([[np.inf]])), axis=0)
                    lb = np.concatenate((lb, -b[0, None, :]), axis=0)

            A = np.delete(A, 0, 0)
            b = np.delete(b, 0, 0)

            nrows = A.shape[0]

        for i in range(self.ne):
            row0 = Ae[i, None, :]
            P = np.concatenate((P, row0))
            ub = np.concatenate((ub, be[i, None, :]))
            lb = np.concatenate((lb, -be[i, None, :]))

        self.check_solution(P, lb, ub)

        if self.check_empty(lb, ub):
            self.P = csc_matrix(np.eye(self.dim))
            self.lb = np.zeros((self.dim, 1))
            self.ub = np.zeros((self.dim, 1))
        else:
            self.P = csc_matrix(P)
            self.ub = ub
            self.lb = lb
        self.has_bounds = True

    def affine_map(self, M, name=None):
        # Check dimensions
        if M.shape[1] != self.dim:
            raise ValueError(
                "The matrix must have %i columns, has %i" % (self.dim, M.shape[1]))

        # If mapping is close to zero
        if np.linalg.norm(M) <= 1e-5:
            raise Warning("This mapping is close to zero")
            return

        # Check if invertible
        if M.shape[0] == M.shape[1] and abs(np.linalg.det(M)) > 1e-8:
            Minv = np.linalg.inv(M)
            S_new = Polyhedron(np.matmul(self.A, Minv), b=self.b, name=name)

        else:  # If matrix is not square, go to V representation
            hej = np.concatenate((self.b, -self.A), axis=1)
            mat = cdd.Matrix(hej.tolist(), number_type='float')
            mat.rep_type = cdd.RepType.INEQUALITY
            poly = cdd.Polyhedron(mat)
            Vrep = cdd.Polyhedron(mat).get_generators()

            # Vrep = self.compute_vrep()
            new_V = np.concatenate((np.array(Vrep)[:, None, 0],
                                    np.matmul(M, np.array(Vrep)[:, 1:].T).T), axis=1)

            new_V[abs(new_V) < 1e-6] = 0
            new_V = np.around(new_V, 6)
            mat = cdd.Matrix(new_V.tolist(), number_type='float')
            mat.rep_type = cdd.RepType.GENERATOR

            try:
                poly = cdd.Polyhedron(mat).get_inequalities()

                A = -np.array(poly)[:, 1:]
                b = np.array(poly)[:, None, 0]

                for i in poly.lin_set:
                    A = np.concatenate((A,
                                        np.array(poly)[i, None, 1:]))

                    b = np.concatenate((b,
                                        -np.array(poly)[i, None, 0, None]))

                S_new = Polyhedron(A, b=b, name=name)
            except:
                print("WARNING NUMERICAL INCONSISTENCY FOUND!")
                print(mat)

                S_new = Polyhedron(A=np.concatenate(
                    (np.eye(self.dim), -np.eye(self.dim))), b=np.zeros((self.dim*2, 1)), name=name)

        return S_new

    def minrep(self):
        P = self.A.copy()
        p = self.b.copy()
        nc = P.shape[0]

        Q = np.empty((0, self.dim))
        q = np.empty((0, 1))
        idx = [i for i in range(nc)]

        settings = {'verbose': False, 'eps_abs': 1e-4,
                    'eps_rel': 1e-4, 'max_iter': int(5e7)}

        for i in range(nc):
            idx.pop(0)
            Pi = P[i, None, 0:]
            pi = p[i, None, :]

            PI = P[idx, 0:]
            pI = p[idx]

            new_P = np.concatenate((PI, Pi), axis=0)
            new_p = np.concatenate((pI, pi+1))

            new_set = Polyhedron(new_P, b=new_p)

            problem = osqp.OSQP()
            problem.setup(P=csc_matrix(np.zeros((self.dim, self.dim))),
                          q=-Pi.T, A=new_set.P,
                          l=new_set.lb, u=new_set.ub, **settings)

            result = problem.solve()

            if -result.info.obj_val > pi:
                idx.append(i)

        PI = P[idx, 0:]
        pI = p[idx]
        new_set = Polyhedron(PI, pI)

        if new_set.A.shape != self.A.shape:
            print("Minrep is NOT THE SAME: different number of variables!")
            self.A = new_set.A
            self.b = new_set.b
            self.lb = new_set.lb
            self.ub = new_set.ub
            self.P = new_set.P
        if not (new_set.A == self.A).all() or not (new_set.b == self.b).all():
            print("Minrep is NOT THE SAME same!!!!!!!!1")
            self.A = new_set.A
            self.b = new_set.b
            self.lb = new_set.lb
            self.ub = new_set.ub
            self.P = new_set.P

        return new_set

    def pontryagin_difference(self, Q, name=None):
        """ Compute the Pontryagin difference P-Q
        P = {y: P*y<= p}
        P = {z: Q*z<= q}
        W = {x: P*x <= p - H(P, Q)}
        H(P, Q) = max_{x in Q} P*x
        """
        problem = osqp.OSQP()
        settings = {'verbose': False, 'eps_abs': 1e-4,
                    'eps_rel': 5e-3, 'max_iter': int(5e3)}

        problem.setup(P=csc_matrix(np.zeros((self.dim, self.dim))),
                      q=-self.A[0, None, 0:].T,
                      A=Q.P, l=Q.lb, u=Q.ub, **settings)

        H = np.zeros((self.ni, 1))
        for i in range(self.ni):
            problem.update(q=-self.A[i, None, 0:].T)

            results = problem.solve()

            if not (results.info.status_val == 1 or results.info.status_val == 2):
                print("ERROR in pontryagin difference: %s" % self.name)
                print("\n\nCOULD NOT SOLVE OPTIMIZATION PROBLEM:\nSTATUS_VAL = ",
                      results.info.status_val, utils.return_status_values(results.info.status_val))
                print("MINIMIZE:")
                print(-self.A[i, None, 0:].T)

                print("SUBJECT TO")
                print(Q)
                print("x = ", results.x.T)
                raise

            H[i, :] = -results.info.obj_val

        W = Polyhedron(self.A, self.b-H, name=name)

        return W
