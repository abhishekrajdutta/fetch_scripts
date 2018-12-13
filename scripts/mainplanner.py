#!/usr/bin/env python

# simple_disco.py: Move the fetch arm through a simple disco motion
import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
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


# Note: fetch_moveit_config move_group.launch must be running
# Safety!: Do NOT run this script near people or objects.
# Safety!: There is NO perception.
#          The ONLY objects the collision detection software is aware
#          of are itself & the floor.
if __name__ == '__main__':
	rospy.init_node("planner")

	move_group = MoveGroupInterface("arm_with_torso", "base_link")
	planning_scene = PlanningSceneInterface("base_link")
	planning_scene.removeCollisionObject("my_front_ground")
	planning_scene.removeCollisionObject("my_back_ground")
	planning_scene.removeCollisionObject("my_right_ground")
	planning_scene.removeCollisionObject("my_left_ground")
	planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
	planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
	planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
	planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

	joint_names = ["torso_lift_joint","shoulder_pan_joint",
	               "shoulder_lift_joint", "upperarm_roll_joint",
	               "elbow_flex_joint", "forearm_roll_joint",
	               "wrist_flex_joint", "wrist_roll_joint"]

	Q = [0,0, 0, 0.0, -np.pi/2, 0.0, np.pi/2, 0.0]
	move_group.moveToJointPosition(joint_names, Q , wait=False)
	move_group.get_move_action().wait_for_result()
# position: 
#       x: 0.640036865808
#       y: -0.0391988119345
#       z: 1.11652027628
#     orientation: 
#       x: 0.00118531019309
#       y: 0.00278754644384
#       z: -0.0268161732665
#       w: 0.999635792414

	L = np.array([[-0.087 + 0.1195,0,0.3486 + 0.3774],[0,0,0],[0.117,0,0.06],[0.219,0,0],[0.133,0,0],[0.197,0,0],[0.1245,0,0],[0.1385,0,0],[0.16645,0,0]])	                   


	g= []
	g.append (SE3(np.eye(3),np.array([L[0][0],L[0][1],L[0][2] + Q[0]])))
	g.append (SE3(Rotz(Q[1]),L[1]))
	g.append (SE3(Roty(Q[2]),L[2]))
	g.append (SE3(Rotx(Q[3]),L[3]))
	g.append (SE3(Roty(Q[4]),L[4]))
	g.append (SE3(Rotx(Q[5]),L[5]))
	g.append (SE3(Roty(Q[6]),L[6]))
	g.append (SE3(Rotx(Q[7]),L[7]))
	g.append (SE3(Rotz(0),L[8]))

	g_curr = np.eye(4)
	for i in range(0,8):
		g_curr = np.matmul(g_curr,g[i])

	si = []
	si.append(np.array([[0,0,0,0,0,1]]).T)
	si.append(np.array([[0,0,1,0,-1*(-L[0][0]),0]]).T)
	si.append(np.array([[0,1,0, -1 * (L[0][2] + L[2][2]),0,(L[0][0]+L[2][0])]]).T)
	si.append(np.array([[1,0,0,0,(L[0][2]+L[2][2]+L[3][2]),0]]).T) #y is always 0
	si.append(np.array([[0,1,0, -1 * (L[0][2] + L[2][2]),0,(L[0][0]+L[2][0] + L[3][0] + L[4][0])]]).T)
	si.append(np.array([[1,0,0,0,(L[0][2]+L[2][2]+L[3][2]),0]]).T) #y is always 0
	si.append(np.array([[0,1,0, -1 * (L[0][2] + L[2][2]),0,(L[0][0]+L[2][0] + L[3][0] + L[4][0] + L[5][0] + L[6][0])]]).T)
	si.append(np.array([[1,0,0,0,(L[0][2]+L[2][2]+L[3][2]),0]]).T) #y is always 0

	# Consider the wrist roll link to be the last link

	goals = np.array([[0.640036865808,0,1.11652027628], [0.640036865808,0,1.11652027628], [0.640036865808,0,1.11652027628], [0.640036865808,0,1.11652027628], [0.640036865808,0,1.11652027628]])

	for i in range (0,4):
		start = getTranslation(g_curr)	
		goal = goals[i]
		diff = goal - start
		diff = diff.astype(float)
		N = 20

		print Q

		for i in range(1,N+1):
			g.append (SE3(np.eye(3),np.array([L[0][0],L[0][1],L[0][2] + Q[0]])))
			g.append (SE3(Rotz(Q[1]),L[1]))
			g.append (SE3(Roty(Q[2]),L[2]))
			g.append (SE3(Rotx(Q[3]),L[3]))
			g.append (SE3(Roty(Q[4]),L[4]))
			g.append (SE3(Rotx(Q[5]),L[5]))
			g.append (SE3(Roty(Q[6]),L[6]))
			g.append (SE3(Rotx(Q[7]),L[7]))
			# g.append (SE3(Rotz(0),L[8]))
			
			g_curr = np.eye(4)
			for k in range(0,8):
				g_curr = np.matmul(g_curr,g[k])

			pose = start + (float(i)/N)*diff
			curr = getTranslation(g_curr)
			v = np.array([0,0,0,pose[0] - curr[0],pose[1] - curr[1],pose[2] - curr[2]]).T

			J = np.zeros((6,8))

			for n in range(0,8):
				if n == 0:
					J[:,n] = np.squeeze(si[n])
				else:
					T = np.eye(4)				
					for m in range(0,n):
						T = np.matmul(T,g[k])					
					d = getTranslation(T)
					R = getR(T)
					AdT = np.r_[np.c_[R,np.zeros((3,3))],np.c_[np.matmul(hat_d(d),R),R,]]
					J[:,n] = np.squeeze(np.matmul(AdT,si[n]))

			Qdot = np.matmul(np.matmul(J.T,np.linalg.inv(np.matmul(J,J.T)+ 0.05*np.eye(6))),v)
			Q = Q + np.squeeze(Qdot)

		print Q

		move_group.moveToJointPosition(joint_names, Q , wait=False)
		move_group.get_move_action().wait_for_result()

