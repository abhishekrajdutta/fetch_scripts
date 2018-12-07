import gtsam
import gtsam.utils.plot as gtsam_plot
from gtsam import Pose2, Pose3, Point3, Rot3
import numpy as np
from functools import reduce

import math

def compose(*poses):
  """Compose all Pose2 transforms given as arguments from left to right."""
  return reduce((lambda x, y: x.compose(y)), poses)


# setup


# [roll, pitch, yaw, x, y, z]
def get_si():
  si = []
  #si.append(np.array([0, 0, 1, 0, 0, 0], dtype=np.float))
  #si.append(np.array([0, 1, 0, 0, 0, 1], dtype=np.float))
  #si.append(np.array([1, 0, 0, 0, 0, 0], dtype=np.float))
  #si.append(np.array([0, 1, 0, 0, 0, 3], dtype=np.float))
  #si.append(np.array([1, 0, 0, 0, 0, 0], dtype=np.float))
  #si.append(np.array([0, 1, 0, 0, 0, 5], dtype=np.float))
  #si.append(np.array([1, 0, 0, 0, 0, 0], dtype=np.float))
  si.append(np.array([1, 0, 0, 0, 0, 0], dtype=np.float))
  si.append(np.array([0, 1, 0, 0, 0, 1], dtype=np.float))
  si.append(np.array([0, 0, 1, 0, 0, 0], dtype=np.float))
  si.append(np.array([0, 1, 0, 0, 0, 3], dtype=np.float))
  si.append(np.array([0, 0, 1, 0, 0, 0], dtype=np.float))
  si.append(np.array([0, 1, 0, 0, 0, 5], dtype=np.float))
  si.append(np.array([0, 0, 1, 0, 0, 0], dtype=np.float))
  return si


def get_T(q, L):
  T = []
  T.append(Pose3(Rot3.Ypr(q[0], 0.0, 0.0), Point3(0, 0, 0)))
  T.append(Pose3(Rot3.Ypr(0.0, q[1], 0.0), Point3(L[0], 0, 0)))
  T.append(Pose3(Rot3.Ypr(0.0, 0.0, q[2]), Point3(L[1], 0, 0)))
  T.append(Pose3(Rot3.Ypr(0.0, q[3], 0.0), Point3(L[2], 0, 0)))
  T.append(Pose3(Rot3.Ypr(0.0, 0.0, q[4]), Point3(L[3], 0, 0)))
  T.append(Pose3(Rot3.Ypr(0.0, q[5], 0.0), Point3(L[4], 0, 0)))
  T.append(Pose3(Rot3.Ypr(0.0, 0.0, q[6]), Point3(L[5], 0, 0)))
  T.append(Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(L[6], 0, 0)))
  return T


def get_expMap(q, si):
  g = []
  for i in range(0, 7):
    # fk = compose(fk,T[i])
    g.append(Pose3.Expmap(q[i] * si[i]))
  return g


def get_Jacobian(T, si):
  J = np.array([[0, 0, 0, 0, 0, 0]]).T
  for i in range(0, 7):
    G = Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(0, 0, 0))
    for j in range(0, i):
      G = compose(G, T[j])
    if i == 0:
      J = np.c_[J, np.expand_dims(si[i], axis=1)]
    else:
      J = np.c_[J, np.expand_dims(G.Adjoint(si[i]), axis=1)]
    print J

  J = np.delete(J, 0, 1)
  return J


def get_fk(L, q):
  T = list()
  T.append(Pose3(Rot3.Ypr(q[0], 0.0, 0.0), Point3(0, 0, 0)))
  T.append(Pose3(Rot3.Ypr(0.0, q[1], 0.0), Point3(L[0], 0, 0)))
  T.append(Pose3(Rot3.Ypr(0.0, 0.0, q[2]), Point3(L[1], 0, 0)))
  T.append(Pose3(Rot3.Ypr(0.0, q[3], 0.0), Point3(L[2], 0, 0)))
  T.append(Pose3(Rot3.Ypr(0.0, 0.0, q[4]), Point3(L[3], 0, 0)))
  T.append(Pose3(Rot3.Ypr(0.0, q[5], 0.0), Point3(L[4], 0, 0)))
  T.append(Pose3(Rot3.Ypr(0.0, 0.0, q[6]), Point3(L[5], 0, 0)))
  T.append(Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(L[6], 0, 0)))

  fk = Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(0, 0, 0))

  for i in range(0, 8):
    fk = compose(fk, T[i])

  return fk


def main():
  
  # Lengths of manipulator links
  lengths = [1, 1, 1, 1, 1, 1, 1]

  #q = [0, 0, 0, 0, 0, 0, math.pi/4]
  q = [0, 0, 0, 0, 0, 0, 0]
  v = np.array([[0, 0, 1, 0, 0, 0]]).T
 
  x_goal = [2, 5, 0]
 
  si = get_si()
  T = get_T(q, lengths)

  # FK init
  fk = Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(0, 0, 0))

  # expMap
  #g = get_expMap(q, si)
  #gtool = Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(7, 0, 0))
  #G = compose(*(g + [gtool]))
  #fk_1 = get_fk(lengths, q)
  #fk_1.between(G)
  # Jacobian
  #check jacobian forward
  #q_dot = np.array([0.5, 0, 1, 0, 0, -1, 0]).T
  #J.dot(q_dot)
  
  J = get_Jacobian(T, si)
  Jinv = np.linalg.pinv(J)
  
  d = np.matmul(Jinv, v)
  q += np.squeeze(d)

  fk = get_fk(lengths, q)
  print fk


if __name__ == '__main__':
  main()
