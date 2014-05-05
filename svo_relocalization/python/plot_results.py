

import numpy as np
import numpy.linalg
from T_plot import *
import matplotlib.cm as cm

if __name__ == '__main__':
  
  test_file_path = '/home/fox/Documents/vibot/masterThesisZurich/relocalizer_1/data/test_reloc_1_2014-04-24_17-41-22/pose.txt'
  results_file_path = '/home/fox/Documents/vibot/masterThesisZurich/tests/test_reloc_1_1/pose.txt'

  test_data = np.loadtxt(test_file_path)
  results_data = np.loadtxt(results_file_path)

  diff = test_data[:,1:4] - results_data[:-1,1:4]
  d = np.linalg.norm(diff, axis=1)
  fig = plt.figure()
  plt.hist(d, 60)
  fig.show()


  fig = plt.figure()
  ax = fig.gca(projection='3d')
  cmap = cm.get_cmap('gist_rainbow')
  for i in range(0, results_data.shape[0]-1, 3):
  #for i in range(results_data.shape[0]-1):
    color = cmap(1.0/results_data.shape[0]*i)
    #color = cmap(np.random.rand())
    #color = [1,0,0]
    T = np.eye(4)
    T[0:3,0:3] = quat2rot(np.mat(test_data[i,4:]).T)
    T[0:3,3] = test_data[i,1:4]

    tmp_coor = Coordinates(T)
    tmp_coor.selfPlot(ax, color)

    #color = [0,0,1]
    T = np.eye(4)
    T[0:3,0:3] = quat2rot(np.mat(results_data[i,4:]).T)
    T[0:3,3] = results_data[i,1:4]

    tmp_coor = Coordinates(T)
    tmp_coor.selfPlot(ax, color)

  ax.plot(test_data[:,1], test_data[:,2], test_data[:,3])
  ax.plot(results_data[:,1], results_data[:,2], results_data[:,3])

  fig.show()

