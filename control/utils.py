import numpy as np


def rref(B, tol=1e-8, debug=False):
    A = B.copy()
    rows, cols = A.shape
    r = 0
    pivots_pos = []
    row_exchanges = np.arange(rows)
    for c in range(cols):
        if debug:
            print("Now at row", r, "and col", c, "with matrix:")
            print(A)

        # Find the pivot row:
        pivot = np.argmax(np.abs(A[r:rows, c])) + r
        m = np.abs(A[pivot, c])
        if debug:
            print("Found pivot", m, "in row", pivot)
        if m <= tol:
            # Skip column c, making sure the approximately zero terms are
            # actually zero.
            A[r:rows, c] = np.zeros(rows-r)
            if debug:
                print("All elements at and below (", r,
                      ",", c, ") are zero.. moving on..")
        else:
            # keep track of bound variables
            pivots_pos.append((r, c))

            if pivot != r:
                # Swap current row and pivot row
                A[[pivot, r], c:cols] = A[[r, pivot], c:cols]
                row_exchanges[[pivot, r]] = row_exchanges[[r, pivot]]

                if debug:
                    print("Swap row", r, "with row", pivot, "Now:")
                    print(A)

            # Normalize pivot row
            A[r, c:cols] = A[r, c:cols] / A[r, c]

            # Eliminate the current column
            v = A[r, c:cols]
            # Above (before row r):
            if r > 0:
                ridx_above = np.arange(r)
                A[ridx_above, c:cols] = A[ridx_above, c:cols] - \
                    np.outer(v, A[ridx_above, c]).T
                if debug:
                    print("Elimination above performed:")
                    print(A)
            # Below (after row r):
            if r < rows-1:
                ridx_below = np.arange(r+1, rows)
                A[ridx_below, c:cols] = A[ridx_below, c:cols] - \
                    np.outer(v, A[ridx_below, c]).T
                if debug:
                    print("Elimination below performed:")
                    print(A)
            r += 1
        # Check if done
        if r == rows:
            break
    return (A, pivots_pos, row_exchanges)


def nilpotent_feedback(A, B, p):
    # Computing a nilpotent feedback that takes the system to the origin in n
    # steps.
    print("* COMPUTE NILPOTENT FEEDBACK")
    K = [None for i in range(p)]

    if A.shape[0] != A.shape[1]:
        print("ERROR: Invalid A matrix\n")
        return
    else:
        nx = A.shape[1]

    if B.shape[0] != nx:
        print("ERROR: Invalid B matrix\n")
        return
    else:
        nu = B.shape[1]

    Btemp = np.zeros((nx, p*nu))
    Atemp = np.linalg.matrix_power(A, p)
    for i in range(0, p):
        Btemp[:, i*nu:(i+1)*nu] = np.matmul(np.linalg.matrix_power(A, i), B)
    Ktemp = -np.matmul(np.linalg.pinv(Btemp), Atemp)

    factor_ = np.eye(nx)
    for i in range(p):
        idx = [(p-i-1)*nu + k for k in range(nu)]
        K[i] = np.matmul(Ktemp[idx, :], np.linalg.inv(factor_))
        factor_ = np.matmul((A + np.matmul(B, K[i])), factor_)

    # Check that system goes to zero
    mat = np.eye(nx)
    for i in range(p):
        mat = np.matmul((A+np.matmul(B, K[i])), mat)

    if (mat > 0.01).any():
        print("Zero matrix = \n", mat)
        raise Warning("Nilpotent did not give expected result")

    return K


def return_status_values(value):
    if value == 1:
        constant = 'solved'
    elif value == 2:
        constant = 'solved inaccurate'
    elif value == -2:
        constant = 'maximum iterations reached'
    elif value == -3:
        constant = 'primal infeasible'
    elif value == 3:
        constant = 'primal infeasible inaccurate'
    elif value == -4:
        constant = 'dual infeasible'
    elif value == 4:
        constant = 'dual infeasible inaccurate'
    elif value == -5:
        constant = 'interrupted by user'
    elif value == -6:
        constant = 'run time limit reached'
    elif value == -7:
        constant = 'problem non convex'
    elif value == -10:
        constant = 'unsolved'
    return constant
