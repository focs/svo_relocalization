
import sys
import numpy as np

if __name__ == '__main__':
  if len(sys.argv) < 2:
    print 'Usage %s <pose_file.txt>' % sys.argv[0]
    sys.exit()


  poses = np.loadtxt(sys.argv[1])
  dists = np.zeros([poses.shape[0]]*2)

  for i in range(poses.shape[0]):
    for j in range(poses.shape[0]):
      dists[i,j] = np.linalg.norm(poses[i,1:4] - poses[j,1:4])
      

  dist_file_name = '/'.join(sys.argv[1].split('/')[:-1])+'/'+'dists.txt'
  print dist_file_name
  np.savetxt(dist_file_name, dists, fmt='%.5f')

