#! /usr/bin/env python

"""
Author: lei.zeng@tu-dortmund.de
"""

import numpy as np
import math
from sklearn.cluster import KMeans
from pandas import DataFrame
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit


class GeometricFitting():
    def __init__(self):
        pass

    def circle_fitting_2d(self, x, y, w=[]):
        A = np.array([x, y, np.ones(len(x))]).T
        b = [ix**2 + iy**2 for (ix, iy) in zip(x, y)]
        if len(w) == len(x):
            W = np.diag(w)
            A = np.dot(W, A)
            b = np.dot(W, b)

        c = np.linalg.lstsq(A, b, rcond=None)[0]

        xc = c[0]/2
        yc = c[1]/2
        r = np.sqrt(c[2]+xc**2+yc**2)

        E = np.dot(A, c) - b
        e = np.sqrt(np.dot(E, E.T) / len(x))
        e_scaled = e/r

        if 0 < len(x) <= 2:
            xc = np.mean(x)
            yc = np.mean(y)
            r = 0.01
            e_scaled = 0.01

        return (xc, yc, r, e_scaled)

    def extract_circles(self, kmean_object):
        for ic in xrange(kmean_object._n_clusters):
            inds = np.array([i for i in range(len(kmean_object._df['x']))
                             if kmean_object._kmeans.labels_[i] == ic])
            x = kmean_object._df['x'][inds]
            y = kmean_object._df['y'][inds]
            cx, cy, cr, cerror = self.circle_fitting_2d(list(x), list(y))
            self.plot_circle(cx, cy, cr, "--g")
            plt.scatter(cx, cy, c='g', s=50, marker='o',
                        alpha=0.8, label="circle center")
            plt.pause(1)

    def plot_circle(self, x, y, size, color="-c"):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color, label="circle fitting")


class KMeansClustering():
    def __init__(self, df, n_clusters):
        self._df = df
        self._n_clusters = n_clusters

    def kmeans_clustering(self):
        self._kmeans = KMeans(self._n_clusters).fit(self._df)
        self._centroids = self._kmeans.cluster_centers_

        plt.scatter(self._df['x'], self._df['y'], c=self._kmeans.labels_.astype(
            float), s=50, alpha=0.3)
        plt.grid(True)
        plt.pause(1)
        plt.scatter(self._centroids[:, 0],
                    self._centroids[:, 1], c='red', s=50, marker='^', alpha=0.8, label="cluster center")
        plt.pause(1)

        for ic in xrange(self._n_clusters):
            inds = np.array([i for i in range(len(self._df['x']))
                             if self._kmeans.labels_[i] == ic])
            x = self._df['x'][inds]
            y = self._df['y'][inds]


def main():
    x_list = [-2.0089040685579214, -1.5970005839323016, -1.247024059885813, -0.9377280541044909, -0.6384461863094921, -0.3363393396548146,
              6.879104818317721, 6.733360653680176, 6.397049354819596, 6.299134595302138, 6.309348871875073, 6.498765743879554,
              12.94605082013413, 11.72104519151878, 11.654421611058371, 11.356675060492377]
    y_list = [-6.182770979565098, -5.960087318952518, -5.866786939382871, -5.920581921955503, -6.074409700957008, -6.417736913413813,
              0.3605186069833004, 0.7077047222730503, 1.0131930817915378, 1.3389223910741088, 1.6905849349851412, 2.1115769913901623,
              4.969523305590242, 5.218545541061846, 5.9382244099721015, 6.556779403274383]

    plt.cla()
    plt.axis("equal")
    plt.scatter(x_list, y_list, c='pink', s=50, alpha=0.8)
    plt.pause(3)

    Data = {'x': x_list,
            'y': y_list
            }
    df = DataFrame(Data, columns=['x', 'y'])

    KM = KMeansClustering(df, 3)
    KM.kmeans_clustering()

    GF = GeometricFitting()
    GF.extract_circles(KM)


if __name__ == '__main__':
    main()
