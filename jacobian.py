
import gtsam
import gtsam.utils.plot as gtsam_plot
from gtsam import Pose2, Pose3, Point3,Rot3
import numpy as np
from functools import reduce



def compose(*poses):
    """Compose all Pose2 transforms given as arguments from left to right."""
    return reduce((lambda x, y: x.compose(y)), poses)

FETCH_WHEEL_RADIUS = 0.055325
FETCH_WHEEL_BASE = 2 * 0.18738

base_jacobian = np.array([[1, 1], #x
													[0, 0], #y
													[0, 0], #z
													[0, 0], #th_x
													[0, 0], #th_y
													[1/FETCH_WHEEL_BASE, -1/FETCH_WHEEL_BASE]]) * FETCH_WHEEL_RADIUS #th_z

si = []
si.append(np.array([0,0,1,0,0,0], dtype=np.float))
si.append (np.array([0,-1,0,0,0,-1], dtype=np.float))
si.append (np.array([1,0,0,0,0,0], dtype=np.float))
si.append (np.array([0,-1,0,0,0,-3], dtype=np.float))
si.append (np.array([1,0,0,0,0,0], dtype=np.float))
si.append (np.array([0,-1,0,0,0,-5], dtype=np.float))
si.append (np.array([1,0,0,0,0,0], dtype=np.float))

# print Pose3(Rot3.rpy(0.0, 0.0, 0), Point3(0, 0, 0))

v = np.array([[0,0,0,0,1,0]]).T
q = [0,0,0,0,0,0,0]
# def get_T(q):
L = [1, 1, 1, 1, 1, 1, 1]
T=[]
T.append(Pose3(Rot3.Ypr(q[0], 0.0, 0.0 ), Point3(0, 0, 0)))
T.append(Pose3(Rot3.Ypr(0.0, q[1], 0.0), Point3(L[0], 0, 0)))
T.append(Pose3(Rot3.Ypr(0.0, 0.0, q[2]), Point3(L[1], 0, 0)))
T.append(Pose3(Rot3.Ypr(0.0, q[3],0.0 ), Point3(L[2], 0, 0)))
T.append(Pose3(Rot3.Ypr(0.0, 0.0 ,q[4] ), Point3(L[3], 0, 0)))
T.append(Pose3(Rot3.Ypr(0.0, q[5],0.0 ), Point3(L[4], 0, 0)))
T.append(Pose3(Rot3.Ypr(0.0, 0.0 ,q[6] ), Point3(L[5], 0, 0)))
T.append(Pose3(Rot3.Ypr(0.0, 0.0 , 0.0 ), Point3(L[6], 0, 0)))

fk = Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(0, 0, 0))


# forward kinematics
g = []
g.append ( Pose3.Expmap(q[0]*np.array([0,0,1,0,0,0], dtype=np.float)))
g.append ( Pose3.Expmap(q[1]*np.array([0,1,0,0,0,1], dtype=np.float)))
g.append ( Pose3.Expmap(q[2]*np.array([1,0,0,0,0,0], dtype=np.float)))
g.append ( Pose3.Expmap(q[3]*np.array([0,1,0,0,0,3], dtype=np.float)))
g.append ( Pose3.Expmap(q[4]*np.array([1,0,0,0,0,0], dtype=np.float)))
g.append ( Pose3.Expmap(q[5]*np.array([0,1,0,0,0,5], dtype=np.float)))
g.append ( Pose3.Expmap(q[6]*np.array([1,0,0,0,0,0], dtype=np.float)))
gtool =  Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(7, 0, 0))

fk = Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(0, 0, 0))
# 
J = np.array([[0,0,0,0,0,0]]).T
for i in range(0,7):
	G = Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(0, 0, 0))
	for j in range (0,i):
		G = compose(G,T[j])

	if i == 0:
		J = np.c_[J,np.expand_dims(si[i], axis=1)]
	else:		
		J = np.c_[J,np.expand_dims(G.Adjoint(si[i]), axis=1)]

J = np.delete(J, 0, 1)
Jinv = np.linalg.pinv(J)
d = np.matmul(Jinv,v)
print J
print Jinv

q = q+np.squeeze(d)

print q

del T[:]
T.append(Pose3(Rot3.Ypr(q[0], 0.0, 0.0 ), Point3(0, 0, 0)))
T.append(Pose3(Rot3.Ypr(0.0, q[1], 0.0), Point3(L[0], 0, 0)))
T.append(Pose3(Rot3.Ypr(0.0, 0.0, q[2]), Point3(L[1], 0, 0)))
T.append(Pose3(Rot3.Ypr(0.0, q[3],0.0 ), Point3(L[2], 0, 0)))
T.append(Pose3(Rot3.Ypr(0.0, 0.0 ,q[4] ), Point3(L[3], 0, 0)))
T.append(Pose3(Rot3.Ypr(0.0, q[5],0.0 ), Point3(L[4], 0, 0)))
T.append(Pose3(Rot3.Ypr(0.0, 0.0 ,q[6] ), Point3(L[5], 0, 0)))
T.append(Pose3(Rot3.Ypr(0.0, 0.0 , 0.0 ), Point3(L[6], 0, 0)))

fk = Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(0, 0, 0))

for i in range(0,8):
	fk = compose(fk,T[i])
print fk


# # test fk
# g.append ( Pose3.Expmap(q[0]*np.array([0,0,1,0,0,0], dtype=np.float)))
# g.append ( Pose3.Expmap(q[1]*np.array([0,1,0,0,0,1], dtype=np.float)))
# g.append ( Pose3.Expmap(q[2]*np.array([1,0,0,0,0,0], dtype=np.float)))
# g.append ( Pose3.Expmap(q[3]*np.array([0,1,0,0,0,3], dtype=np.float)))
# g.append ( Pose3.Expmap(q[4]*np.array([1,0,0,0,0,0], dtype=np.float)))
# g.append ( Pose3.Expmap(q[5]*np.array([0,1,0,0,0,5], dtype=np.float)))
# g.append ( Pose3.Expmap(q[6]*np.array([1,0,0,0,0,0], dtype=np.float)))
# gtool =  Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(7, 0, 0))

# fk = Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(0, 0, 0))

# for i in range(0,7):
# 	fk = compose(fk,g[i])

# print compose(fk,gtool)