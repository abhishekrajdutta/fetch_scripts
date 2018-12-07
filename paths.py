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


# [yaw, pitch, roll, x, y, z]
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
  si.append(np.array([0, 1, 0, 1, 0, 0], dtype=np.float))
  si.append(np.array([0, 0, 1, 0, 0, 0], dtype=np.float))
  si.append(np.array([0, 1, 0, 3, 0, 0], dtype=np.float))
  si.append(np.array([0, 0, 1, 0, 0, 0], dtype=np.float))
  si.append(np.array([0, 1, 0, 5, 0, 0], dtype=np.float))
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
    # print J

  J = np.delete(J, 0, 1)
  return J


def get_fk(L, q):
  T = get_T(q, L)
  fk = Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(0, 0, 0))
  for i in T:
    fk = compose(fk, i)
  return fk

def delta(current,goal):
  return np.array([goal.translation().x() - current.translation().x() ,goal.translation().y() - current.translation().y(),goal.translation().z() - current.translation().z()])

def interpolate (current, goal, N=400):
  diff = delta(current, goal)
  return [Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(current.translation().x() + diff[0]*t, current.translation().y() + diff[1]*t, current.translation().z() + diff[2]*t)) for t in np.linspace(0, 1, N)]

def clamp_to_circle(val):
  if val < -2*math.pi:
    return val % (-2*math.pi)
  elif val > 2*math.pi:
    return val % (2*math.pi)
  else:
    return val

def main():
  
  # Lengths of manipulator links
  lengths = [1, 1, 1, 1, 1, 1, 1]

  #q = [0, 0, 0, 0, 0, 0, math.pi/4]
  
  # v = np.array([[0, 0, 1, 0, 0, 0]]).T
  q = [0, 0, 0, 0, 0, 0, 0]
 
  si = get_si()
  T = get_T(q, lengths)

  # FK init
  fk = Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(0, 0, 0))

  goal = Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(2, 2, 2))
  
  current = get_fk(lengths, q)

  poses = interpolate(current,goal)
    
  for pose in poses:
    current = get_fk(lengths, q)
    error = delta(current,pose)
    if np.sum(delta(current, goal)**2) < 0.1 and sum(q) > 0:
        print "Woah!"
        print current
        break
    T = get_T(q,lengths)
    J = get_Jacobian(T,si)
    
    pinv_J = np.linalg.pinv(J)
    #thresh_inds = pinv_J < 1e-4
    #pinv_J[thresh_inds] = 0.
    
    val_pre = np.squeeze(np.matmul(pinv_J, np.array([[0,0,0,error[0],error[1],error[2]]]).T))
    ctc_vec = np.vectorize(clamp_to_circle)
    val = ctc_vec(val_pre)
    q += val
    print "\n-------------------------------------- current"
    print current
    print "\n", error.tolist()
    print "\n=================== val"
    print val.tolist()
    print "\n******************* q"
    print q.tolist()

  # print get_fk(lengths,q)

if __name__ == '__main__':
  np.set_printoptions(suppress=True)
  main()
