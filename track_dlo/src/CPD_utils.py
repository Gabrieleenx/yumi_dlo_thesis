import numpy as np
import numpy.matlib as mlib
import math



def cpd_normalize(x, y):
    '''
    Normalizes the points
    :param x: points of goal position
    :param y: points of current position
    :return: normalized points
    '''

    normal = {}
    [n, d] = np.shape(x)
    [m, d] = np.shape(y)

    normal['xd'] = np.mean(x,axis=0)
    normal['yd'] = np.mean(y,axis=0)

    x = x - np.transpose(np.repeat(normal['xd'][:, np.newaxis], n, axis=1))
    y = y - np.transpose(np.repeat(normal['yd'][:, np.newaxis], m, axis=1))

    normal['xscale'] = np.sqrt(sum(sum(np.power(x,2), 2)) / n)
    normal['yscale'] = np.sqrt(sum(sum(np.power(y,2), 2)) / m)

    x_norm = x / normal['xscale']
    y_norm = y / normal['yscale']

    return x_norm, y_norm, normal

def cpd_G(x,y,beta):
    '''
    Constructs gaussian affinity matrix
    :param x: goal positions
    :param y: current positions
    :param beta: std of G
    :return: Gaussian affinity matrix G
    '''
    k = -2 * beta ** 2
    [n, d] = np.shape(x)
    [m, d] = np.shape(y)

    G = np.tile(x,[m,1,1]) - np.reshape(np.repeat(y[:, :, np.newaxis], n, axis=0), [m, n, d])
    G = np.sum(np.power(G,2), 2)
    G = G / k

    return np.exp(G)








