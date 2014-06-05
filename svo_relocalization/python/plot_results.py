#!/usr/bin/python2

import sys
import yaml
import numpy as np
import numpy.linalg
from T_plot import *
import matplotlib.cm as cm

if __name__ == '__main__':
  
  #test_file_path = '/home/fox/Documents/vibot/masterThesisZurich/datasets/desktop_2_2/frames_data.yaml'
  #results_file_path = '/home/fox/Documents/vibot/masterThesisZurich/datasets/results/frames_data_desktop1_CC_3pt.yaml'

  if len(sys.argv) >= 3:
    test_file_path = sys.argv[1]
    results_file_path = sys.argv[2]

  yaml_test_frames = yaml.load(open(test_file_path, 'r'))
  yaml_results_frames = yaml.load(open(results_file_path, 'r'))

  test_data = np.zeros((len(yaml_test_frames['frames']), 7))
  for i in range(len(yaml_test_frames['frames'])):
    test_data[i] = yaml_test_frames['frames'][i]['pose']

  results_data = np.zeros((len(yaml_results_frames['frames']), 7))
  for i in range(len(yaml_results_frames['frames'])):
    results_data[i] = yaml_results_frames['frames'][i]['pose']

  print 'Done reading'

  #test_data = np.loadtxt(test_file_path)
  #results_data = np.loadtxt(results_file_path)

  print 'Shape test data', test_data.shape
  print 'Shape results data', results_data.shape

  print test_data
  print 'Results'
  print results_data

  diff = test_data - results_data
  d = np.linalg.norm(diff, axis=1)

  fig = plt.figure()
  plt.hist(d, bins=50)
  plt.axvline(x=np.mean(d), color='r')
  fig.show()

  fig = plt.figure()
  ax = fig.gca(projection='3d')
  cmap = cm.get_cmap('gist_rainbow')
  #for i in range(0, results_data.shape[0]-1, 3):
  for i in range(results_data.shape[0]):
    color = cmap(1.0/results_data.shape[0]*i)
    color = cmap(np.random.rand())
    #color = [1,0,0]
    T = np.eye(4)
    T[0:3,0:3] = quat2rot(np.mat(test_data[i,3:]).T)
    T[0:3,3] = test_data[i,0:3]

    tmp_coor = Coordinates(T)
    tmp_coor.selfPlot(ax,color)

    #color = [0,0,1]
    T = np.eye(4)
    T[0:3,0:3] = quat2rot(np.mat(results_data[i,3:]).T)
    T[0:3,3] = results_data[i,0:3]

    tmp_coor = Coordinates(T)
    tmp_coor.selfPlot(ax, color)

  ax.plot(test_data[:,0], test_data[:,1], test_data[:,2])
  ax.plot(results_data[:,0], results_data[:,1], results_data[:,2])

  fig.show()
  a = input("Press a key")
