# Importing packages
import cv2
import imutils
import numpy as np
import math
import random

# import ros stuff
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from tf.transformations import euler_from_quaternion


x_pos = 0
y_pos = 0
yaw = 0


def clbk_pose(msg):
    global x_pos, y_pos, yaw
    
    x_pos = msg.pose.pose.position.x
    y_pos = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

class Nodes:
    #Class to store the RRT graph

    def __init__(self, x,y):
        self.x = x
        self.y = y
        self.parent_x = []
        self.parent_y = []


def collision(img,x1,y1,x2,y2):
	# check collision
    d = 0.0
    color=[]
    
    try:

        x = list(np.arange(x1,x2,(x2-x1+d)/100))
        y = list(((y2-y1)/(x2-x1+d))*(x-x1+d) + y1)

    except:

        print("Error in collision!!!")
        print("X1: ",x1)
        print("X2: ",x2)
        print("Y1: ",y1)
        print("Y2: ",y2)

    # print("collision",x,y)
    for i in range(len(x)):
        # print(int(x[i]),int(y[i]))
        color.append(img[int(y[i]),int(x[i])])
    if (0 in color):
        return True #collision
    else:
        return False #no-collision


def check_collision(img,x1,y1,x2,y2):
	# check the  collision with obstacle and trim
    _,theta = dist_and_angle(x2,y2,x1,y1)
    x=x2 + stepSize*np.cos(theta)
    y=y2 + stepSize*np.sin(theta)
    # print(x2,y2,x1,y1)
    # print("theta",theta)
    # print("check_collision",x,y)

    # TODO: trim the branch if its going out of image area
    # print("Image shape",img.shape)
    hy,hx=img.shape
    if y<0 or y>hy or x<0 or x>hx:
        # print("Point out of image bound")
        directCon = False
        nodeCon = False
    else:
        # check direct connection
        if collision(img,x,y,end[0],end[1]):
            directCon = False
        else:
            directCon=True

        # check connection between two nodes
        if collision(img,x,y,x2,y2):
            nodeCon = False
        else:
            nodeCon = True

    return(x,y,directCon,nodeCon)

# return dist and angle b/w new point and nearest node
def dist_and_angle(x1,y1,x2,y2):
    dist = math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )
    angle = math.atan2(y2-y1, x2-x1)
    return(dist,angle)

# return the neaerst node index
def nearest_node(x,y):
    temp_dist=[]
    for i in range(len(node_list)):
        dist,_ = dist_and_angle(x,y,node_list[i].x,node_list[i].y)
        temp_dist.append(dist)
    return temp_dist.index(min(temp_dist))

# generate a random point in the image space
def rnd_point(h,l):
    new_y = random.randint(0, h)
    new_x = random.randint(0, l)
    return (new_x,new_y)


def RRT(img, img2, start, end, stepSize):
    h,l= img.shape # dim of the loaded image

    # insert the starting point in the node class
    # node_list = [0] # list to store all the node points         
    node_list[0] = Nodes(start[0],start[1])
    node_list[0].parent_x.append(start[0])
    node_list[0].parent_y.append(start[1])

    i=1
    pathFound = False
    while pathFound==False:
        nx,ny = rnd_point(h,l)
        # print("Random points:",nx,ny)
        nearest_ind = nearest_node(nx,ny)
        nearest_x = node_list[nearest_ind].x
        nearest_y = node_list[nearest_ind].y
        # print("Nearest node coordinates:",nearest_x,nearest_y)

        #check direct connection
        tx,ty,directCon,nodeCon = check_collision(img,nx,ny,nearest_x,nearest_y)
        # print("Check collision:",tx,ty,directCon,nodeCon)

        if directCon and nodeCon:
            # print("Node can connect directly with end")
            node_list.append(i)
            node_list[i] = Nodes(tx,ty)
            node_list[i].parent_x = node_list[nearest_ind].parent_x.copy()
            node_list[i].parent_y = node_list[nearest_ind].parent_y.copy()
            node_list[i].parent_x.append(tx)
            node_list[i].parent_y.append(ty)

            cv2.circle(img2, (int(tx),int(ty)), 2,(0,0,255),thickness=3, lineType=8)
            cv2.line(img2, (int(tx),int(ty)), (int(node_list[nearest_ind].x),int(node_list[nearest_ind].y)), (0,255,0), thickness=1, lineType=8)
            cv2.line(img2, (int(tx),int(ty)), (end[0],end[1]), (255,0,0), thickness=2, lineType=8)
            print("Path has been found")
            #print("parent_x",node_list[i].parent_x)
            len_path = len(node_list[i].parent_x)
            path_x = np.zeros(len_path)
            path_y = np.zeros(len_path)
            # print('Tamnho: ', len_path)
            for j in range(len(node_list[i].parent_x)-1):
                cv2.line(img2, (int(node_list[i].parent_x[j]),int(node_list[i].parent_y[j])), (int(node_list[i].parent_x[j+1]),int(node_list[i].parent_y[j+1])), (255,0,0), thickness=2, lineType=8)
                (path_x[j],path_y[j]) = pixel2gunit((node_list[i].parent_x[j],node_list[i].parent_y[j]))
                if j == len(node_list[i].parent_x)-2:
                    (path_x[j+1],path_y[j+1]) = pixel2gunit((end[0],end[1]))
            cv2.imwrite("media/"+str(i)+".jpg",img2)
            cv2.imwrite("out.jpg",img2)
            # print()
            break

        elif nodeCon:
            # print("Nodes connected")
            node_list.append(i)
            node_list[i] = Nodes(tx,ty)
            node_list[i].parent_x = node_list[nearest_ind].parent_x.copy()
            node_list[i].parent_y = node_list[nearest_ind].parent_y.copy()
            # print(i)
            # print(node_list[nearest_ind].parent_y)
            node_list[i].parent_x.append(tx)
            node_list[i].parent_y.append(ty)
            i=i+1
            # display
            cv2.circle(img2, (int(tx),int(ty)), 2,(0,0,255),thickness=3, lineType=8)
            cv2.line(img2, (int(tx),int(ty)), (int(node_list[nearest_ind].x),int(node_list[nearest_ind].y)), (0,255,0), thickness=1, lineType=8)
            cv2.imwrite("media/"+str(i)+".jpg",img2)
            #cv2.imshow("sdc",img2)
            #cv2.waitKey(1)
            continue

        else:
            # print("No direct con. and no node con. :( Generating new rnd numbers")
            continue 
    return path_x, path_y

def pixel2gunit(pos_pixel):
    # Convert image coordenates to gazebo coordenates    
    # print(pos_pixel)
    pos_gunit = np.zeros(2)
    pos_gunit[0] = 1 + pos_pixel[0]*0.05
    pos_gunit[1] = 30 - (1 + pos_pixel[1]*0.05)
    return pos_gunit


def img_prepossessing(img, start, end):
	# Prepossessing to use in the RRT

	cropped_map = img[1400:2050,1800:2600] # first crop on maze
	rot_map = imutils.rotate(cropped_map,-2) # rotate to align the maze 
	gazebo_map = rot_map[81:637, 147:710] # second crop
	gray_map = cv2.cvtColor(gazebo_map, cv2.COLOR_BGR2GRAY) # transform map in grayscale
	_, bw_inv_map = cv2.threshold(gray_map, 253, 255, cv2.THRESH_BINARY_INV) # transform map in black and white and invert
	kernel = np.ones((30, 30), 'uint8')
	bigwall_map = cv2.dilate(bw_inv_map, kernel, iterations=1) # make walls bigger
	_, bw_map = cv2.threshold(bigwall_map, 253, 255, cv2.THRESH_BINARY_INV) # invert again

	cv2.circle(gazebo_map, (start[0],start[1]), 5,(54, 0, 102),thickness=3, lineType=8) # plot start point
	cv2.circle(gazebo_map, (end[0],end[1]), 5,(54, 0, 102),thickness=3, lineType=8) # plot end point
	
	# show results
	# cv2.imshow('Pospossessed Image', gazebo_map)
	# cv2.waitKey(0)

	return bw_map, gazebo_map

if __name__ == '__main__':

    img = cv2.imread('mapa.pgm') # load maze
    start_gunit = (13,6) # starting coordinate in gazebo
    end_gunit = (7,23) # target coordinate in gazebo
    start = (int((start_gunit[0]-1)/0.05),int((-start_gunit[1]+29)/0.05)) # starting coordinate in picture
    end = (int((end_gunit[0]-1)/0.05),int((-end_gunit[1]+29)/0.05)) # target coordinate in picture
    stepSize = 20 # stepsize for RRT
    node_list = [0] # list to store all the node points

    bw_maze,maze = img_prepossessing(img, start, end)

    path_x, path_y = RRT(bw_maze, maze, start, end, stepSize)

    #pathGazebo = [[13.0, 5.385999999999999], [13.5, 5.385999999999999], [14.0, 4.885999999999999], [14.5, 4.785999999999998], [15.0, 4.785999999999998], [15.5, 4.785999999999998], [16.0, 4.785999999999998], [16.5, 4.785999999999998], [17.0, 4.785999999999998], [17.5, 4.785999999999998], [18.0, 4.785999999999998], [18.5, 4.785999999999998], [19.0, 4.785999999999998], [19.5, 4.785999999999998], [20.0, 4.785999999999998], [20.200000000000003, 3.8859999999999992], [20.200000000000003, 3.3859999999999992], [20.200000000000003, 2.8859999999999992], [20.200000000000003, 2.3859999999999992], [19.900000000000002, 2.0859999999999985], [19.400000000000002, 2.0859999999999985], [18.900000000000002, 2.0859999999999985], [18.400000000000002, 2.0859999999999985], [17.900000000000002, 2.0859999999999985], [17.400000000000002, 2.0859999999999985], [16.9, 2.0859999999999985], [16.4, 2.0859999999999985], [15.9, 2.0859999999999985], [15.4, 2.0859999999999985], [14.9, 2.0859999999999985], [14.4, 2.0859999999999985], [13.9, 2.0859999999999985], [13.4, 2.0859999999999985], [12.9, 2.0859999999999985], [12.4, 2.0859999999999985], [11.9, 2.0859999999999985], [11.4, 2.0859999999999985], [10.9, 2.0859999999999985], [10.4, 2.0859999999999985], [9.9, 2.0859999999999985], [9.700000000000001, 2.5859999999999985], [9.700000000000001, 3.0859999999999985], [9.700000000000001, 3.5859999999999985], [9.700000000000001, 4.0859999999999985], [9.700000000000001, 4.5859999999999985], [9.700000000000001, 5.0859999999999985], [9.700000000000001, 5.5859999999999985], [9.700000000000001, 6.0859999999999985], [9.700000000000001, 6.5859999999999985], [9.700000000000001, 7.0859999999999985], [9.700000000000001, 7.5859999999999985], [9.700000000000001, 8.085999999999999], [10.0, 9.085999999999999], [10.5, 9.085999999999999], [11.0, 9.085999999999999], [11.5, 9.085999999999999], [12.0, 9.686], [12.5, 9.686], [13.0, 10.186], [13.5, 10.686], [14.0, 11.186], [14.5, 11.485999999999997], [15.0, 11.485999999999997], [15.5, 11.485999999999997], [16.0, 11.485999999999997], [16.5, 11.485999999999997], [17.0, 11.485999999999997], [17.5, 11.485999999999997], [18.0, 11.485999999999997], [18.5, 11.485999999999997], [19.0, 11.485999999999997], [19.5, 11.485999999999997], [20.0, 11.485999999999997], [20.5, 11.585999999999999], [21.0, 12.085999999999999], [21.1, 12.585999999999999], [21.1, 12.585999999999999], [21.1, 13.085999999999999], [21.1, 13.585999999999999], [21.1, 14.085999999999999], [21.1, 14.585999999999999], [21.1, 15.085999999999999], [21.1, 15.585999999999999], [21.1, 16.086], [21.1, 16.586], [21.1, 17.086], [21.1, 17.586], [21.1, 18.086], [21.1, 18.586], [21.1, 19.086], [21.1, 19.586], [21.1, 20.086], [21.1, 20.586], [21.1, 21.086], [21.1, 21.586], [21.1, 22.086], [21.0, 22.586], [20.5, 23.186], [20.0, 23.186], [19.5, 23.186], [19.0, 23.186], [18.5, 22.286], [18.0, 21.786], [17.5, 21.286], [17.0, 20.786], [16.5, 20.286], [16.4, 19.786], [16.4, 19.286], [16.4, 19.286], [16.4, 18.785999999999998], [15.9, 18.086], [15.4, 18.086], [14.9, 18.086], [14.4, 18.086], [13.9, 18.086], [13.4, 18.086], [12.9, 18.086], [12.4, 18.086], [11.9, 18.086], [11.4, 18.086], [10.9, 18.086], [10.4, 18.086], [9.9, 18.086], [9.4, 18.886], [8.9, 18.886], [8.4, 19.386], [7.9, 19.886], [7.4, 20.386], [7.0, 20.886], [7.0, 21.386], [7.0, 21.886], [7.0, 22.386]]
    
    # print(path_x)
    # print(path_y)

    global pub_
    global target

    rospy.init_node('amcl_pose')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    sub_pose = rospy.Subscriber('/odom', Odometry, clbk_pose)

    e = 0.5
    K = 5
    target = 1

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        
        msg = Twist()
        dist,ang = dist_and_angle(x_pos,y_pos,path_x[target],path_y[target])

        print('Distancia: ', dist)

        if dist < e:
            target = target +1
            dist,ang = dist_and_angle(x_pos,y_pos,path_x[target],path_y[target])
        
        e_ang = ang - yaw

        print('Erro angulo: ', e_ang)
        print('Caminho: ' , target)
        print('Orientação: ', yaw)
        print('Angulo do Alvo:', ang)
        print('Pos robo: ', x_pos, ' || ', y_pos)
        print('Pos alvo: ', path_x[target], ' || ', path_y[target])

        if abs(e_ang) < np.deg2rad(5):
            msg.linear.x = 0.5
            msg.angular.z = 0.0

        else:
            msg.angular.z = K*np.sin(e_ang)
            msg.linear.x = 0.0
        
        pub_.publish(msg)

        rate.sleep()

        
