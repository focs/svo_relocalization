import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt


def vecExpancion(vec):
  """@todo: Docstring for vecExpancion.

  :vec: @todo
  :returns: @todo

  """
  m = np.zeros((3,3))
  m[0,1] = -vec[2]
  m[0,2] = vec[1]
  m[1,0] = vec[2]
  m[1,2] = -vec[0]
  m[2,0] = -vec[1]
  m[2,1] = vec[0]
  
  return m

def quat2rot(quat):
  R = (quat[3]**2 - quat[0:3].T*quat[0:3])[0,0]*np.eye(3) + 2* quat[0:3]*quat[0:3].T + 2*quat[3,0]*vecExpancion(quat[0:3])

  return R


class Coordinates(object):

  """Docstring for Coordinates. """

  def __init__(self, T):
    """@todo: to be defined1.

    :T: @todo

    """
    X = np.mat([[0.,0,0], [0.1,0,0]]).T
    Y = np.mat([[0.,0,0], [0,0.1,0]]).T
    Z = np.mat([[0.,0,0], [0,0,0.1]]).T

    self.X_ = self.projectPoint(X,T)
    self.Y_ = self.projectPoint(Y,T)
    self.Z_ = self.projectPoint(Z,T)


  def projectPoint(self, p, T):
    """@todo: Docstring for projectPoint.

    :p: @todo
    :T: @todo
    :returns: @todo

    """

    p_tmp = np.concatenate((p, np.ones((1,p.shape[1])) ),0)

    p_T = T*p_tmp

    return p_T[0:3,:]/p_T[-1,:]

  def selfPlot(self, ax, c=[0,0,1]):
    """@todo: Docstring for selfPlot.

    :ax: @todo
    :returns: @todo

    """
    ax.plot(self.X_[0,:].tolist()[0], self.X_[1,:].tolist()[0], self.X_[2,:].tolist()[0], color=c)
    ax.plot(self.Y_[0,:].tolist()[0], self.Y_[1,:].tolist()[0], self.Y_[2,:].tolist()[0], color=c)
    ax.plot(self.Z_[0,:].tolist()[0], self.Z_[1,:].tolist()[0], self.Z_[2,:].tolist()[0], color=c)

    #theta = np.linspace(-4 * np.pi, 4 * np.pi, 100)
    #z = np.linspace(-2, 2, 100)
    #r = z**2 + 1
    #x = r * np.sin(theta)
    #y = r * np.cos(theta)
    #ax.plot(x, y, z)

if __name__ == '__main__':

  template = np.mat([[-0.957863,-0.245896, 0.148437,-0.203502],
 [-0.18463, 0.923005, 0.337601,-0.068085],
[-0.220022,  0.29597,-0.929512, 0.207077],
        [0,        0,        0,        1]])

  query = np.mat([[-0.954564,-0.256421, 0.151843,-0.190091],
[-0.200414, 0.929469, 0.309712,-0.086056],
[-0.22055, 0.265208,-0.938628, 0.204064],
[       0,        0,        0,        1]])

  T = np.mat([[ -0.955737, -0.261019,  0.135777, -0.186407],
[-0.211252,   0.92999,  0.300817,-0.0855124],
[ -0.20479,  0.258819, -0.943967,  0.207659],
[        0,         0,         0,         1]])

  found = np.mat([[0.99986,-0.00901424,   0.014119,          0],
[0.00886103,   0.999902,  0.0108766,          0],
[-0.0142157,   -0.01075,   0.999841,          0],
[         0,          0,          0,          1]])


  #T = query*found
  

  original_coor = Coordinates(np.eye(4))
  template_coor = Coordinates(template)
  query_coor = Coordinates(query)
  T_coor = Coordinates(T)
  found_coor = Coordinates(found)

  fig = plt.figure()
  ax = fig.gca(projection='3d')
  T_coor.selfPlot(ax)
  template_coor.selfPlot(ax)
  query_coor.selfPlot(ax)
  found_coor.selfPlot(ax)
  original_coor.selfPlot(ax)

  fig.show()
