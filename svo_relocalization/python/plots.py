#!/usr/bin/python2

import sys
import os.path
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as ptl
import matplotlib as mpl
import matplotlib.cm

pose_file_name = 'pose.txt'
dist_file_name = 'dist.txt'
confusion_matrix_file_name = 'confusion_matrix.txt'


def distMatrix(path):
  """
  If the distance matrix exists, it is read from the file.
  Otherwise it is created and writen to a file

  :path: path to the dataset folder
  :returns: distance matrix

  """
  dist_file = path + dist_file_name
  if os.path.isfile(dist_file):
    dists = np.loadtxt(dist_file)
  else:
    print('Dist file not found, creating it....')
    poses = np.loadtxt(path + pose_file_name)
    dists = np.zeros([poses.shape[0]] * 2)

    for i in range(poses.shape[0]):
      for j in range(poses.shape[0]):
        dists[i, j] = np.linalg.norm(poses[i, 1:4] - poses[j, 1:4])

    np.savetxt(dist_file, dists, fmt='%.5f')

  return dists


def plotDistDiff(dists, confusion_matrix):
  """@todo: Docstring for plotDistDiff

  :dists: @todo
  :confusion_matrix: @todo
  :returns: @todo

  """
  # flater matrices
  confusion_matrix_vec = confusion_matrix.ravel()
  dists_vec = dists.ravel()

  # Sort dist (x coord)
  sort_idx = np.argsort(dists_vec)
  dists_vec_sort = dists_vec[sort_idx]
  confusion_matrix_vec_sort = confusion_matrix_vec[sort_idx]

  # Init interpolation
  inter = interp1d(dists_vec_sort, confusion_matrix_vec_sort)

  # Create new x vector
  x = np.linspace(dists_vec_sort[0], dists_vec_sort[-1:], 100)

  f = ptl.figure()
  ptl.imshow(dists, cmap=mpl.cm.get_cmap('gray'))
  f.show()

  f = ptl.figure()
  ptl.plot(x, inter(x))
  ptl.plot(dists_vec_sort)
  f.show()

if __name__ == '__main__':

  if len(sys.argv) < 2:
    print('Usage: %s <dataset path>' % sys.argv[0])
    sys.exit()

  path = sys.argv[1]
  confusion_matrix_file = path + confusion_matrix_file_name

  # Load data
  confusion_matrix = np.loadtxt(confusion_matrix_file)
  dists = distMatrix(path)

  # Display confusion matrix
  f = ptl.figure()
  ptl.imshow(confusion_matrix, cmap=mpl.cm.get_cmap('gray'))
  f.show()
  #ptl.show()
  #ptl.matshow(confusion_matrix)

  plotDistDiff(dists, confusion_matrix)

  a = input("Press a key")
