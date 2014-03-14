
import sys
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as ptl

if __name__ == '__main__':

  if len(sys.argv) < 3:
    print 'Usage: %s <confusion_matrix_file.txt> <dist_file.txt>' % sys.argv[0]
    sys.exit()
  
  confusion_matrix_file = sys.argv[1]
  dist_file = sys.argv[2]

  confusion_matrix = np.loadtxt(confusion_matrix_file)
  dists = np.loadtxt(dist_file)

  confusion_matrix_vec = confusion_matrix.reshape(confusion_matrix.size)
  dists_vec = dists.reshape(dists.size)

  sort_idx = np.argsort(dists_vec)
  dists_vec_sort = dists_vec[sort_idx]
  confusion_matrix_vec_sort = confusion_matrix_vec[sort_idx]

  inter = interp1d(dists_vec_sort, confusion_matrix_vec_sort)

  x = np.linspace(dists_vec_sort[0], dists_vec_sort[-1:], 100)
  print  inter(x)
  ptl.plot(x, inter(x))
  ptl.show()



