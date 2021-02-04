import numpy as np
from scipy.sparse import spdiags
from numpy.linalg import inv
from scipy.linalg import schur
import CPD_utils
import scipy
import time
import math
from numpy import linalg

class SPR(object):
    def __init__(self):
        self.Phi = 0
        self.seq = 0

    def SPR_register(self, X, Y, opt):
        """
        :params:
        :return
        """

        M, D = Y.shape
        N, D2 = X.shape

        # Check the input options and set the defaults
        if 'method' not in opt or opt.get('method') is None:
            opt['method'] = "rigid"
        if 'normalize' not in opt or opt.get('normalize') is None:
            opt['normalize'] = True
        if 'max_it' not in opt or opt.get('max_it') is None:
            opt['max_it'] = 150
        if 'tol' not in opt or opt.get('tol') is None:
            opt['tol'] = 1e-5
        if 'viz' not in opt or opt.get('viz') is None:
            opt['viz'] = True
        if 'corresp' not in opt or opt.get('corresp') is None:
            opt['corresp'] = 0
        if 'outliers' not in opt or opt.get('outliers') is None:
            opt['outliers'] = 0.1
        if 'fgt' not in opt or opt.get('fgt') is None:
            opt['fgt'] = False
        if 'sigma2' not in opt or opt.get('sigma2') is None:
            opt['sigma2'] = 0

        # strictly rigid params
        if 'rot' not in opt or opt.get('rot') is None:
            opt['rot'] = 1
        if 'scale' not in opt or opt.get('scale') is None:
            opt['scale'] = 1

        # strictly non-rigid params
        if 'beta' not in opt or opt.get('beta') is None:
            opt['beta'] = 2
        if 'lambda' not in opt or opt.get('lambda') is None:
            opt['lambda'] = 3
        if 'knn' not in opt or opt.get('knn') is None:
            opt['knn'] = 5
        if 'tau' not in opt or opt.get('tau') is None:
            opt['tau'] = 3

        # lowrank app param
        if 'numeig' not in opt or opt.get('numeig') is None:
            opt['numeig'] = round(np.sqrt(M))
        if 'eigfgt' not in opt or opt.get('eigfgt') is None:
            opt['eigfgt'] = 1

        # checking for the possible errors
        if D != D2:
            exit("The dimension of point-sets is not the same.")
        if (D > M) or (D > N):
            print('The dimensionality is larger than the number of points. Possibly the wrong orientation of X and Y.')
        if (M > 1e+5) or (N > 1e+5) and not opt.get('fgt'):
            print('The data sets are large. Use opt.fgt=1 to speed up the process.')
        if (M > 1e+5) or (N > 1e+5) and (opt.get('method').lower() is 'nonrigid'):
            print('The data sets are large. Use opt.method=''nonrigid_lowrank'' to speed up the non-rigid registration')
        if (D <= 1) or (D > 3):
            opt['viz'] = False
        if (opt.get('normalize')) and (opt.get('scale') == 0):
            opt['scale'] = 1

        Yorig = Y

        # default mean and scaling
        normal = {'xd': 0, 'yd': 0, 'xscale': 1, 'yscale': 1}

        # Normalize to zero mean and unit variance
        if opt.get('normalize'):
            print('normalize')
            [X, Y, normal] = CPD_utils.cpd_normalize(X, Y)
        if self.seq == 0:
            M, D = Y.shape
            self.Phi = LocalReg(np.transpose(Y), opt.get('knn'), D)
            self.seq = 1
            
        #if opt.get('viz'):
        print('%%%%% Starting SPR-', opt.get('method').upper(), ' registration. %%%')
            # tic

        # Choose the method and start SPR point - set registration
        method = opt.get('method').lower()
        if method == 'nonrigid':
            [W, sigma2, iter, T] = cpd_MLLE(X, Y, opt.get('beta'), opt.get('lambda'),
                                            opt.get('max_it'), opt.get('tol'), opt.get('viz'), opt.get('outliers'),
                                            opt.get('fgt'), opt.get('corresp'), opt.get('sigma2'), opt.get('knn'),
                                            opt.get('tau'), opt.get('tau_annealing_handle'),
                                            opt.get('lambda_annealing_handle'), self.Phi)
            '''
            elif method == 'nonrigid_lowrank':
                [C, W, sigma2, iter, T] = cpd_MLLE_lowrank(X, Y, opt.get('beta'),
                                                        opt.get('lambda'), opt.get('max_it'), opt.get('tol'),
                                                        opt.get('viz'), opt.get('outliers'), opt.get('fgt'),
                                                        opt.get('numeig'), opt.get('eigfgt'), opt.get('corresp'),
                                                        opt.get('sigma2'), opt.get('knn'), opt.get('tau'))
            '''
        else:
            exit('The opt.method value is invalid. Supported methods are: nonrigid, nonrigid_lowrank')

        # if opt.viz, disptime(toc); end;

        Transform = {'iter': iter, 'method': opt.get('method'), 'Y': T, 'normal': normal}

        # Denormalize transformation parameters
        Transform['beta'] = opt.get('beta')
        Transform['W'] = W
        Transform['Yorig'] = Yorig
        Transform['s'] = 1
        Transform['t'] = np.zeros(D)

        if opt.get('normalize'):
            Transform['Y'] = np.dot(T, normal.get('xscale')) + np.tile(normal.get('xd'), (M,1))

        return Transform


# Define function cpd_MLLE
def cpd_MLLE(X, Y, beta, lambda0, max_it, tol, viz, outliers,
             fgt, corresp, sigma2, knn, tau0, tau_annealing_handle,
             lambda_annealing_handle, Phi):
    """
    :params:
    :return
    """

    N, D = X.shape
    M, D = Y.shape

    # Initialization
    iter = 0
    ntol = tol + 10
    W = np.zeros([M, D])
    T = Y
    L = 1

    if 'sigma2' not in locals() or not sigma2 or sigma2 == 0:
        sigma2 = (M * np.trace(np.dot(np.transpose(X), X)) +
                  N * np.trace(np.dot(np.transpose(Y), Y)) - 2
                  * np.dot(np.sum(X, 0), np.transpose(np.sum(Y, 0)))) / (M * N * D)
    sigma2_init = sigma2

    # Construct affinity matrix G
    G = CPD_utils.cpd_G(Y, Y, beta)


    while (iter < max_it) and (ntol > tol) and (sigma2 > 7.7e-4):
        time_start = time.time()

        # Simulated Annealing
        tau_annealing_ratio = tau_annealing_handle(iter, max_it)

        tau = tau0 * tau_annealing_ratio

        lambda_annealing_ratio = lambda_annealing_handle(iter, max_it)
        lambda_var = lambda0 * lambda_annealing_ratio

        L_old = L

        time_global_st = time.time()
        # Check whether use the Fast Gauss Transform
        if not fgt:         # CPD without FGT
            [P1, Pt1, PX] = GlobalReg(X, T, sigma2, outliers)
            st = ""
        else:               # CPD with FGT
            [P1, Pt1, PX, L, sigma2, st] = CPD_utils.cpd_Pfast(X, T, sigma2, outliers, sigma2_init, fgt)

        time_global = time.time()-time_global_st
        # THE L SEEMS TO BE WRONG (SKIP THIS FOR NOW)
        #L = L + lambda_var / 2 * np.trace(np.dot(np.dot(np.transpose(W), G), W)) \
        #    + tau / 2 * np.trace(np.dot(np.dot(np.transpose(T), Phi), T))

        L = L + lambda_var / 2 * np.trace(np.transpose(W).dot(G).dot(W)) \
            + tau / 2 * np.trace(np.transpose(T).dot(Phi).dot(T))

        ntol = abs((L - L_old) / L)

        # Solve linear system for W.
        time_solve_st = time.time()
        dP1 = spdiags(P1, diags=0, m=M, n=M).toarray()

        W = scipy.linalg.solve(np.dot(dP1, G) + (lambda_var * sigma2 * np.eye(M)) + tau * sigma2 * np.dot(Phi, G),
            (PX - np.dot(dP1, Y) - tau * sigma2 * np.dot(Phi, Y)))
        time_solve = time.time() - time_solve_st
        # Update the new position of the image Y
        time_update_st = time.time()
        T = Y + np.dot(G, W)

        Np = np.sum(P1, 0)
        sigma2save = sigma2

        sigma2 = np.abs((np.sum(np.multiply(pow(X, 2), np.transpose(np.tile(Pt1, (D,1)))))
                        + np.sum(np.multiply(pow(T, 2), np.transpose(np.tile(P1, (D,1)))))
                        - 2 * np.trace(np.dot(np.transpose(PX), T))) / (Np * D))
        time_update = time.time() - time_update_st 
        iter += 1

    if viz:
        print('SPR registration succesfully completed: iter ' + str(iter)
              + ', ntol: ' + str(ntol) + ', sigma2: ' + str(sigma2))

    return W, sigma2, iter, T


# Function MLLE
def LocalReg(X, k, d):
    """
    :params:
    :return
    """

    N = X.shape[1]  # X: m x N, m dimension, N points
    tol = 1.0e-3  # the regularlization parameter

    # Neighborhood selection
    X2 = np.sum(pow(X, 2), axis=0)
    D = np.tile(X2, (N, 1)) + np.transpose(np.tile(X2, (N, 1))) - 2 * np.dot(np.transpose(X), X)
    J = D.argsort()

    # Initialize variables
    ratio = np.zeros(N)
    Theta = []
    Ev = []
    Ow = []

    # Local Information
    for i in range(N):
        xi = X[:, i]
        Ii = J[i, 0:k+1]
        Ji = Ii[1::]
        Gi = X[:, Ji] - np.transpose(np.tile(xi, (k,1)))
        [S, V] = schur(np.dot(np.transpose(Gi), Gi), output='real')
        ei = np.sort(np.diag(S))[::-1]
        JIi = np.diag(S).argsort()[::-1]
        ratio[i] = np.sum(ei[d:k]) / np.sum(ei[0:d])
        Theta.append(V[:, JIi])                 # The local coordinates system
        Ev.append(ei)
        C = np.dot(np.transpose(Gi),Gi)
        C = C + np.eye(k) * tol * np.trace(C)   # Regularization
        Cw = np.linalg.solve(C, np.ones([k, 1]))
        Cw = Cw/np.sum(Cw)
        Ow.append(Cw)                           # Considered as the optimal weight vector

    temp = np.sort(ratio)
    eta = temp[math.ceil(N / 2)]

    # Determine the number of weights to use
    s = np.zeros(N)
    for i in range(N):
        ell = k - d
        Lambda = Ev[i]
        while np.sum(Lambda[k - ell::]) / np.sum(Lambda[0: k - ell]) > eta and ell > 1:
            ell = ell - 1
        s[i] = ell


    Phi = np.zeros((N, N))
    for i in range(N):
        Ii = J[i, 0:k+1]
        Vi = np.array(Theta[i])
        Ve = Vi[:, int(k-s[i])::]
        ve = np.sum(Ve, axis=0)
        alpha = linalg.norm(ve) / np.sqrt(s[i])
        u = ve - alpha
        normu = linalg.norm(u)

        if normu > 1.0e-5:
            u = u / normu
            Wi = ((1 - alpha) ** 2) * np.array(Ow[i]) * np.ones(int(s[i]))\
                + (2 - alpha) * (Ve - np.dot(np.dot(Ve, (2 * u))[:, None], np.transpose(u[:, None])))     # the multiple local weights
        else:
            Wi = ((1-alpha) ** 2) * np.array(Ow[i]) * np.ones(int(s[i]))\
                + (2 - alpha) * Ve

        Phi_add = np.dot(np.vstack((-np.ones(int(s[i])), Wi)),
                         np.transpose(np.vstack((-np.ones(int(s[i])), Wi))))
        for i in range(Phi_add.shape[0]):
            Phi[Ii, Ii[i]] = Phi[Ii, Ii[i]] + Phi_add[:, i]

    return Phi

def GlobalReg(X, T, sigma2, outliers):
    """
    :params:
    :return
    """
    [N, D] = X.shape
    M = T.shape[0]

    # Calculate P matrix
    # Nominator of P
    P_num = np.sum((X[None, :, :] - T[:, None, :])**2, axis=2)
    P_num = np.exp(-P_num / (2 * sigma2))
    # Denominator of P
    P_den = np.sum(P_num, axis=0)
    P_den = np.tile(P_den, (M, 1))
    P_den[P_den == 0] = 2.220446049250313e-16
    c = ((((2 * np.pi * sigma2) ** D / 2) * (outliers / (1 - outliers))) * (M / N))
    P_den += c

    P = np.divide(P_num, P_den)

    P1 = np.sum(P, axis=1)
    Pt1 = np.sum(P, axis=0)

    c1 = c * np.ones(N)
    K1 = np.dot(np.transpose(P_num), np.ones(M))
    a = np.tile(np.divide(1, K1 + c1).reshape(N, 1), D)
    Px = np.dot(P_num, (np.multiply(a, X)))

    return P1, Pt1, Px

