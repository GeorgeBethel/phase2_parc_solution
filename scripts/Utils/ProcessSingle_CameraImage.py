#!/usr/bin/env python

import math
import roslib
import time
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image



mask = 0
detected_edges = 0
cropped_image = 0
hsv = 0

cv_image = 0

def camera_callback(data):

    bridge_object = CvBridge()

    cv_image = bridge_object.imgmsg_to_cv2(data, desired_encoding="passthrough")
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
    # mask white lane
    mask = cv2.inRange(hsv, (0, 0, 119), (0, 0, 132))
    # detected_edges = detect_edges(mask)
    cropped_image = getRegionOfInterest(mask)

    #get original image height
    img_width= mask.shape[1]

    # divide lane into two

    #get left lane image
    left_img = mask[:, 0:img_width/2]

    #get right lane image
    right_img = mask[:, img_width/2:]

    # detected_lines = detect_line_segments(right_img_orig,cv_image)

    vis = np.concatenate((left_img, right_img), axis=1)


     # find the center of the left lane  
    row_sumLeft=np.matrix(np.sum(left_img,0)) #sumes the colunms of image array and gives a row matrix
    row_numbLeft=np.matrix(np.arange(640/2))  
    row_multiplyLeft=np.multiply(row_sumLeft, row_numbLeft)#np.multiply does an elementwise multiplication
    total_pixel_left=np.sum(row_multiplyLeft)
    sum_rowLeft=np.sum(np.sum(left_img)) #sumes all the pixels in the red only matrix

    pixel_centerLeft=total_pixel_left/sum_rowLeft

    # find center of right lane
    row_sumRight=np.matrix(np.sum(right_img,0)) #sumes the colunms of image array and gives a row matrix
    row_numbRight=np.matrix(np.arange(640/2))  
    row_multiplyRight=np.multiply(row_sumRight, row_numbRight)#np.multiply does an elementwise multiplication
    total_pixel_right=np.sum(row_multiplyRight)
    sum_rowRight=np.sum(np.sum(right_img)) #sumes all the pixels in the red only matrix

    pixel_centerRight=total_pixel_right/sum_rowRight

    #print lane centers
    rospy.loginfo("left lane centerX: %s right lane centerX: %s",pixel_centerLeft, pixel_centerRight)

    # print(img.shape)

    # #show images
    # cv2.imshow("original image",cv_image)
    # cv2.imshow("hsv image",hsv)
    # cv2.imshow("detected edge",detected_edges)
    # cv2.imshow("cropped image",cropped_image)
    cv2.imshow("mask image",cropped_image)

    # #save images
    # cv2.imwrite("detected_edges.jpg", detected_edges)
    # cv2.imwrite("cropped_image.jpg", cropped_image)
    # cv2.imwrite("original_image.jpg", cv_image)
    # cv2.imwrite("mask_image.jpg",mask)

    cv2.waitKey(1)



def detect_edges(image):
    
    edges = cv2.Canny(image, 100, 150) 
    return edges

def getRegionOfInterest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus one half of the lower screen
    # specify the coordinates of 4 points to crop out >> lower left, upper left, upper right, lower right
    polygon = np.array([[
        (0, height), 
        (0,  height/1.7),
        (width , height/1.7),
        (width , height),
        ]], np.int32)

    cv2.fillPoly(mask, polygon, 255) # fill the polygon with blue color 
    cropped_edges = cv2.bitwise_and(edges, mask)
    return cropped_edges

def detect_line_segments(cropped_edges,frame):
    rho = 1  
    theta = np.pi / 180  
    min_threshold = 10 
	
	#detects the line amd returns it in the format [x1, y1, x2, y2 ] 
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold, np.array([]), minLineLength=4.5, maxLineGap=5)
    # rospy.loginfo("line coordinates: %s",line_segments[[-1]])
    if(line_segments !=[]):
        for x in range(0, len(line_segments)):
            for x1,y1,x2,y2 in line_segments[x]:
                cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),2)
			    #calculates the angle of the slope in radian
                theta = theta+math.atan2((y2-y1),(x2-x1))   
        
        return theta


def main():
    rospy.init_node("single_image_processing_node",anonymous=False)
    image_sub = rospy.Subscriber("/camera/color/image_raw",Image,camera_callback)

    try:
        rospy.spin()

    except:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':

    main()
    