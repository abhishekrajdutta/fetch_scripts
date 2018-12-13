from graphics import *
import numpy as np
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import rospy

def main():

	markerPub = rospy.Publisher('drawing', Marker, queue_size=10)
	rospy.init_node('stateNode', anonymous=True)

	r = rospy.Rate(1000)

	win = GraphWin("My window",640,480)
	win.setBackground(color_rgb(255,255,255))

	x1=100
	y1=100

	x2=450
	y2=400

	rect = Rectangle(Point(x1,y1),Point(x2,y2))
	rect.setOutline(color_rgb(0,0,0))

	# aimed resolution = 1600x1200

	m = Marker()
	scaled_x1 = x1*2.5
	scaled_y1 = y1*2.5

	scaled_x2 = x2*2.5
	scaled_y2 = y2*2.5

	m.header.frame_id = "world"
	m.ns = "lines"
	m.id = 0

	m.scale.x = 0.1
	m.scale.y = 0.1
	m.scale.z = 0.1

	m.color.r = 0.0
	m.color.g = 1.0
	m.color.b = 0.0
	m.color.a = 1.0

	m.type = Marker.SPHERE
	m.action = Marker.ADD
	
	e1 = np.linspace (scaled_x1,scaled_x2,100)

	markerPub.publish(m)



	for x in e1:
		tform = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,5],[0,0,0,1]])
		vec = np.matmul(tform,np.array([[x/1000,scaled_y1/1000,0,1]]).T)
		m.pose.position.x = vec[0] 
		m.pose.position.y = vec[1] 
		m.pose.position.z = vec[2]
		m.id+=1 

		markerPub.publish(m)
		r.sleep()


	rect.draw(win)

	# win.getMouse()
	win.close()
	rospy.spin()

main()