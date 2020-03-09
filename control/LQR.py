import numpy as np
from scipy import linalg as la
from .Polyhedron import Polyhedron


def get_LQR_infinite_cost(A, B, Q, R, q=None):
    P = Q

    for i in range(800):
        Pnew = Q + np.matmul(np.matmul(A.T, P), A) - \
            np.matmul(np.matmul(np.matmul(np.matmul(A.T, P), B),
                                np.linalg.inv(R + np.matmul(np.matmul(B.T, P), B))),
                      np.matmul(np.matmul(B.T, P), A))

        # print(Pnew)
        P = Pnew

    # P2 = Q + np.matmul(np.matmul(A.T, P), A) - \
    #    np.matmul(np.matmul(np.matmul(np.matmul(A.T, P), B),
    #                        np.linalg.inv(R + np.matmul(np.matmul(B.T, P), B))),
    #              np.matmul(np.matmul(B.T, P), A))

    K = -np.matmul(np.linalg.inv(R + np.matmul(np.matmul(B.T, P), B)),
                   np.matmul(np.matmul(B.T, P), A))
    qf = np.zeros((len(q), 1))

    if q is not None:
        for i in range(800):
            qf += np.matmul(q.T, np.linalg.matrix_power(A +
                                                        np.matmul(B, K), i)).T

    return P, qf


def get_lqr_feedback(A, B, Q, R, q=None):
    # u = K*x
    P, qf = get_LQR_infinite_cost(A, B, Q, R, q)
    K = -np.matmul(np.linalg.inv(R + np.matmul(np.matmul(B.T, P), B)),
                   np.matmul(np.matmul(B.T, P), A))

    return K, P, qf


def pre(X, A):
    print(X[0])
    Xnew = [None, X[1]]
    Xnew[0] = np.matmul(X[0], A)
    return Xnew


def get_invariant_set(A, B, C, D, K, Y):
    Acl = A + np.matmul(B, K)
    nx = A.shape[0]

    # Find initial set Linf s.t. Kx satisfies Y
    print("CONSTRAINTS")
    Y2 = Y.A @ np.concatenate((C, D), axis=1)
    new = np.concatenate((np.eye(nx), K), axis=0)

    print(Y2)
    print(new)

    L = Polyhedron(np.matmul(Y2, new), Y.b)
    print(L.A.shape)
    L.minrep()
    print("INITIAL SET L = ")
    print(L.A)
    print(L.b)
    print(L.A.shape)
    X0 = L

    for i in range(500):
        X1 = X0.pre(Acl).intersection(L)
        #X1 = pre(X0, Acl)
        print(X0.A.shape)
        print(X1.A.shape)

        if (X1.contains(X0)):
            print("CONVERGED")
            print(X1)
        else:
            X0 = X1

    dADDA
