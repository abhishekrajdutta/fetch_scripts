
import gtsam
import gtsam.utils.plot as gtsam_plot
from gtsam import Pose2, Pose3, Point3,Rot3
import numpy as np
from functools import reduce



def compose(*poses):
    """Compose all Pose2 transforms given as arguments from left to right."""
    return reduce((lambda x, y: x.compose(y)), poses)

q = [0,0,0,0,0,0,0]

si = []
si.append(np.array([0,0,1,0,0,0], dtype=np.float))
si.append (np.array([0,1,0,0,0,1], dtype=np.float))
si.append (np.array([1,0,0,0,0,0], dtype=np.float))
si.append (np.array([0,1,0,0,0,3], dtype=np.float))
si.append (np.array([1,0,0,0,0,0], dtype=np.float))
si.append (np.array([0,1,0,0,0,5], dtype=np.float))
si.append (np.array([1,0,0,0,0,0], dtype=np.float))

# link 1
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

for i in range(0,7):
	fk = compose(fk,g[i])


# print fk.Adjoint(np.array([0,0,1,0,0,0], dtype=np.float))

J = np.array([[0,0,0,0,0,0]]).T
print J
for i in range(0,7):
	G = Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(0, 0, 0))
	for j in range (0,i):
		G = compose(G,g[j])

	if i == 0:
		J = np.c_[J,np.expand_dims(si[i], axis=1)]
	else:		
		J = np.c_[J,np.expand_dims(G.Adjoint(si[i]), axis=1)]

print J
