# Importing packages

import cv2
import imutils
import numpy as np
import math
import random

class Nodes:
    #Class to store the RRT graph

    def __init__(self, x,y):
        self.x = x
        self.y = y
        self.parent_x = []
        self.parent_y = []

def pixel2gunit(pos_pixel):
	# Convert image coordenates to gazebo coordenates    

    pos_gunit = []
    pos_gunit[0] = 1 + x_pixel*0.05
    pos_gunit[1] = 30 - (1 + y_pixel*0.05)
    return pos_gunit


def img_prepossessing(img, start, end):
	# Prepossessing to use in the RRT

	cropped_map = img[1400:2050,1800:2600] # first crop on maze
	rot_map = imutils.rotate(cropped_map,-2) # rotate to align the maze 
	gazebo_map = rot_map[81:637, 147:710] # second crop
	gray_map = cv2.cvtColor(gazebo_map, cv2.COLOR_BGR2GRAY) # transform map in grayscale
	_, bw_inv_map = cv2.threshold(gray_map, 253, 255, cv2.THRESH_BINARY_INV) # transform map in black and white and invert
	kernel = np.ones((12, 12), 'uint8')
	bigwall_map = cv2.dilate(bw_inv_map, kernel, iterations=1) # make walls bigger
	_, bw_map = cv2.threshold(bigwall_map, 253, 255, cv2.THRESH_BINARY_INV) # invert again

	cv2.circle(gazebo_map, (start[0],start[1]), 5,(54, 0, 102),thickness=3, lineType=8) # plot start point
	cv2.circle(gazebo_map, (end[0],end[1]), 5,(54, 0, 102),thickness=3, lineType=8) # plot end point
	
	# show results
	cv2.imshow('Pospossessed Image', gazebo_map)
	cv2.waitKey(0)

	return bw_map


if __name__ == '__main__':

	img = cv2.imread('mapa.pgm') # load maze
	start_gunit = (13,6) # starting coordinate in gazebo
	end_gunit = (7,23) # target coordinate in gazebo
	start = (int((start_gunit[0]-1)/0.05),int((-start_gunit[1]+29)/0.05)) # starting coordinate in picture
	end = (int((end_gunit[0]-1)/0.05),int((-end_gunit[1]+29)/0.05)) # target coordinate in picture
	stepSize = 40 # stepsize for RRT
	node_list = [0] # list to store all the node points

	maze = img_prepossessing(img, start, end)
	
