#!/usr/bin/env python

import math
import roslib
import time
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image, LaserScan
import message_filters
from nav_msgs.msg import Odometry

pixel_centerLeft = 0
pixel_centerRight = 0

debug = True
show_frame = True
x_pos = 0

theta = 0

class Lane_follower:

    def __init__(self):

        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.camera_callback)
        rospy.Subscriber("/odom", Odometry, self.OdometryCallback)
        self.vel_comand = Twist()

        #publish to /cmd_vel topic
        self.cmd_velPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #subscribe to laser scan
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.LaserCallback)


    def OdometryCallback(self, pos):

        global x_pos

        x_pos = pos.pose.pose.position.x



    def LaserCallback(self,range):
        global pixel_centerLeft, pixel_centerRight
        global theta

        #wall distance to stick to- < 0.29
        #gets the min distance of the laser data from the angles we are intrested in
        second_quadratLaser = min(range.ranges[0:90])   # center front
        first_quadratLaser = min(range.ranges[90:180])   # left
        third_quadratLaser = min(range.ranges[310:360])  # 
        # fourth_quadratLaser = min(range.ranges[331:360])
        

        if debug == True:

                rospy.loginfo("distanceL: %s distanceCenter: %s distanceRight: %s",first_quadratLaser, second_quadratLaser, third_quadratLaser)
                # rospy.loginfo("left lane centerX: %s right lane centerX: %s theta: %s",pixel_centerLeft, pixel_centerRight, theta)
                # rospy.loginfo("x position: %s", x_pos)


                # rospy.loginfo("distanceB: %s distanceL: %s distanceCenter: %s",first_quadratLaser, second_quadratLaser, third_quadratLaser)
                
        else:
            pass

        self.vel_comand.linear.x = 1.0

        if third_quadratLaser > 0.34 and second_quadratLaser > 0.34:
            self.vel_comand.linear.x = 0.3
            self.vel_comand.angular.z = 0.0

        elif third_quadratLaser < 0.34 and second_quadratLaser > 0.34:
            self.vel_comand.linear.x = 0.3
            self.vel_comand.angular.z = -0.1

        elif third_quadratLaser > 0.34 and second_quadratLaser < 0.34:
            self.vel_comand.linear.x = 0.2
            self.vel_comand.angular.z = 0.5
        
        else:

            self.vel_comand.linear.x = 0.0
            self.vel_comand.angular.z = 0.5


        self.cmd_velPub.publish(self.vel_comand)

    def camera_callback(self,data):

        global pixel_centerLeft, pixel_centerRight, theta
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="passthrough")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # mask white lane
            mask = cv2.inRange(hsv, (0, 0, 198), (180, 252, 255))
            detected_edges = self.detect_edges(mask)
            cropped_image = self.getRegionOfInterest(detected_edges)
            theta = self.detect_line_segments(cropped_image, cv_image)

            #get original image width
            img_width= mask.shape[1]

            ################################## divide lane into two #####################################################################
            #   * if the center of pixel of the left lane increases towards the right, then the robot is heading off the left lane.   ###
            #                                                                                                                         ###
            #   * if the center of pixel of the right lane image is increasing towards the right, then the robot is                   ###
            #     heading off the right lane                                                                                          ###
            #############################################################################################################################

            #get left lane image
            left_img = mask[:, 0:img_width/2]

            #get right lane image
            right_img = mask[:, img_width/2:]

            # combined image
            image_combine = np.concatenate((left_img, right_img), axis=1)


            # find the center of the left lane
            row_sumLeft=np.matrix(np.sum(left_img,0)) #sumes the colunms of image array and gives a row matrix
            row_numbLeft=np.matrix(np.arange(640/2))
            row_multiplyLeft=np.multiply(row_sumLeft, row_numbLeft) #np.multiply does an elementwise multiplication
            total_pixel_left=np.sum(row_multiplyLeft)
            sum_rowLeft=np.sum(np.sum(left_img)) #sumes all the pixels in the red only matrix

            pixel_centerLeft=total_pixel_left/sum_rowLeft  # center of pixel of leftlane

            # find center of right lane
            row_sumRight=np.matrix(np.sum(right_img,0)) #sumes the colunms of image array and gives a row matrix
            row_numbRight=np.matrix(np.arange(640/2))
            row_multiplyRight=np.multiply(row_sumRight, row_numbRight)   #np.multiply does an elementwise multiplication
            total_pixel_right=np.sum(row_multiplyRight)
            sum_rowRight=np.sum(np.sum(right_img))     #sums all the pixels in the red only matrix

            pixel_centerRight=total_pixel_right/sum_rowRight

            if show_frame == True:
                cv2.imshow("detected lane window",cv_image)
                cv2.imshow("detected edge window",detected_edges)
            else:
                pass

            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

    def detect_edges(self,image):

        edges = cv2.Canny(image, 100, 150)
        return edges

    def getRegionOfInterest(self,edges):
        height, width = edges.shape
        mask = np.zeros_like(edges)

        # only focus one half of the lower screen
        # specify the coordinates of 4 points to crop out >> lower left, upper left, upper right, lower right
        polygon = np.array([[
        (0, height),
        (0,  height/1.2),
        (width , height/1.2),
        (width , height),
        ]], np.int32)

        cv2.fillPoly(mask, polygon, 255) # fill the polygon with blue color
        cropped_edges = cv2.bitwise_and(edges, mask)
        return cropped_edges

    def detect_line_segments(self,cropped_edges,frame):
        rho = 1
        theta = np.pi / 180
        min_threshold = 10

	    #detects the line amd returns it in the format [x1, y1, x2, y2 ]
        line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold, np.array([]), minLineLength=4.5, maxLineGap=5)
        if(line_segments !=[]):
                for x in range(0, len(line_segments)):
                    for x1,y1,x2,y2 in line_segments[x]:
                        cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),2)
			            #calculates the angle of the slope in radian
                        theta = theta+math.atan2((y2-y1),(x2-x1))

        return theta

def main():
    rospy.init_node("lane_follower_node",anonymous=False)
    Lane_follower()

    try:
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
