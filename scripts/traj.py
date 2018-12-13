import numpy as np

def Rotz(a):
	return np.array([[np.cos(a),-1*np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])

def Rotx(a):
	return np.array([[1,0,0],[0, np.cos(a),-1*np.sin(a)],[ 0, np.sin(a),np.cos(a)]])

def Roty(a):
	return np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-1*np.sin(a),0,np.cos(a)]])

def SE3(R,t):
	# return np.array(np.r_[np.c_[R,t],np.array([[0,0,0,1]])])
	# print R,t
	return np.r_[np.c_[R,t],np.array([[0,0,0,1]])]

def getTranslation(g):
	return g[0:3,3]

def getR(g):
	return g[0:3,0:3]

def hat_d(d):
	return np.array([[0, -1*d[2], d[1]],[d[2], 0, d[1]],[-1*d[1], d[0], 0]])

def main():
	L = np.array([[1,0,0],[1,0,0],[1,0,0],[1,0,0],[1,0,0],[1,0,0],[1,0,0]])	
	A = np.array([0,0,0,0,0,0,0])
	g= []
	g.append (SE3(Rotz(A[0]),np.array([0,0,0])))
	g.append (SE3(Roty(A[1]),L[0]))
	g.append (SE3(Rotx(A[2]),L[1]))
	g.append (SE3(Roty(A[3]),L[2]))
	g.append (SE3(Rotx(A[4]),L[3]))
	g.append (SE3(Roty(A[5]),L[4]))
	g.append (SE3(Rotx(A[6]),L[5]))
	g.append (SE3(Rotz(0),L[6]))

	g_curr = np.eye(4)
	for i in range(0,8):
		g_curr = np.matmul(g_curr,g[i])

	si = []
	si.append(np.array([[0,0,0,0,0,1]]).T)
	si.append(np.array([[0,0,1,0,1,0]]).T)
	si.append(np.array([[0,0,0,1,0,0]]).T)
	si.append(np.array([[0,0,3,0,1,0]]).T)
	si.append(np.array([[0,0,0,1,0,0]]).T)
	si.append(np.array([[0,0,5,0,1,0]]).T)
	si.append(np.array([[0,0,0,1,0,0]]).T)

	goal = [6.0,0.0,1.0]
	start = getTranslation(g_curr)	
	diff = goal - start
	diff = diff.astype(float)
	N = 200
	# print type(diff),(2.0/N)*diff

	for i in range(1,N+1):
		g[0] = SE3(Rotz(A[0]),np.array([0,0,0]))
		g[1] = SE3(Roty(A[1]),L[0])
		g[2] = SE3(Rotx(A[2]),L[1])
		g[3] = SE3(Roty(A[3]),L[2])
		g[4] = SE3(Rotx(A[4]),L[3])
		g[5] = SE3(Roty(A[5]),L[4])
		g[6] = SE3(Rotx(A[6]),L[5])
		g[7] = SE3(Rotz(0),L[6])		

		g_curr = np.eye(4)
		for k in range(0,8):
			g_curr = np.matmul(g_curr,g[k])

		pose = start + (float(i)/N)*diff
		curr = getTranslation(g_curr)
		v = np.array([[pose[0] - curr[0],pose[1] - curr[1],pose[2] - curr[2],0,0,0]]).T
		
		# Jacobian of size mxn  6x7

		J = np.zeros((6,7))

		for n in range(0,7):
			T = np.eye(4)
			for k in range(n+1,8):
				T = np.matmul(T,g[k])
			d = getTranslation(T)
			R = getR(T)
			AdT = np.r_[np.c_[R,np.matmul(hat_d(d),R)],np.c_[np.zeros((3,3)),R]]
			J[:,n] = np.squeeze(np.matmul(np.linalg.inv(AdT),si[n]))

		Adot = np.matmul(np.matmul(J.T,np.linalg.inv(np.matmul(J,J.T)+ 0.05*np.eye(6))),v)
		A =A + np.squeeze(Adot)

		print getTranslation(g_curr)

	

if __name__ == '__main__':
	main()