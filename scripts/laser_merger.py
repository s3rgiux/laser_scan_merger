#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import math
import tf2_ros
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Header

MIN_ANGLE = -3.12413907051
MAX_ANGLE = 3.14159274101
STEPS = 2880 #360*4 #1440
ANGLE_INCREMENT = float(1/STEPS) #resolution
TIME_INCREMENT = 0.003 
RANGE_MIN = 0.35
RANGE_MAX = 50



class laser_merger:

    def __init__(self):
        self.laser_merged_publisher = rospy.Publisher("/scan_merged", LaserScan, queue_size = 1)
        self.laser_debug = rospy.Publisher("/scan_debug", LaserScan, queue_size = 1)
        self.pcl_debug = rospy.Publisher("/pcl_debug", PointCloud, queue_size = 1)
        self.laser1_sub = rospy.Subscriber('/scan_main', LaserScan, self.callbackLaserMain, queue_size = 1)
        self.laser2_sub = rospy.Subscriber('/scan_aux', LaserScan, self.callbackLaserAux, queue_size = 1)
        self.min_ang = MIN_ANGLE
        self.max_ang = MAX_ANGLE
        self.angle_increment = ANGLE_INCREMENT
        self.time_increment = TIME_INCREMENT
        self.laser_merged = LaserScan()
        self.laser_tmp1 = LaserScan()
        self.laser_tmp2 = LaserScan()
        self.laser_merged.angle_min = self.min_ang
        self.laser_merged.angle_max = self.max_ang
        self.laser_merged.angle_increment = 0.00034722222222 #self.angle_increment
        self.laser_merged.time_increment = self.time_increment
        self.laser_merged.range_min = RANGE_MIN
        self.laser_merged.range_max = RANGE_MAX
        
        self.laser_merged.ranges = np.zeros(STEPS)
        self.laser_merged.intensities = 47 * np.ones(STEPS)
        self.received_laser1 = False
        self.received_laser2 = False
        self.processing = False
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.header = Header()
        self.header.frame_id = "laser"
        self.pcl_obj = PointCloud()
        self.pcl_obj.header = self.header



    def callbackLaserAux(self,msg):
        TRANSLATION_X_AUX = 0.0
        TRANSLATION_Y_AUX = -0.20 #-0.20
        ANGLE_ROTATION_AUX = 0.0
        
        self.received_laser2 = True
        #if not self.processing:
        

        arrayXY2 = self.convertLaserRangesToXY(msg)
        transformedXY2 = self.apply_transformation(arrayXY2 , TRANSLATION_X_AUX , TRANSLATION_Y_AUX , ANGLE_ROTATION_AUX)
        newranges2 = self.convertPointsToLaserRanges(transformedXY2, msg) #self.convertXYtoLAserRanges(transformedXY2, msg)
        msg = self.replaceLaserRanges(newranges2, msg)
        msg.header.stamp = rospy.Time.now()
        self.laser_debug.publish(msg)

        self.laser_tmp2 = msg

        #arrayXY2 = self.convertLaserRangesToXY(self.laser_tmp2)
        #transformedXY2 = self.apply_transformation(arrayXY2 , TRANSLATION_X_AUX , TRANSLATION_Y_AUX , ANGLE_ROTATION_AUX)
        #newranges2 = self.convertPointsToLaserRanges(transformedXY2, self.laser_tmp2) #self.convertXYtoLAserRanges(transformedXY2, self.laser_tmp2)
        #self.laser_tmp2 = self.replaceLaserRanges(newranges2, self.laser_tmp2)
        #self.laser_tmp2.header.stamp = rospy.Time.now()
        #self.laser_debug.publish(self.laser_tmp2)

    def callbackLaserMain(self,msg):
        TRANSLATION_X_MAIN = 0.0
        TRANSLATION_Y_MAIN = 0.0
        ANGLE_ROTATION_MAIN = 0.0

        
        #if not self.processing:
        self.laser_tmp1 = msg

        self.received_laser1 = True
        self.processing = True
        #ApplyTransformations

        #arrayXY1 = self.convertLaserRangesToXY(self.laser_tmp1)
        #transformedXY1 = self.apply_transformation(arrayXY1 , TRANSLATION_X_MAIN , TRANSLATION_Y_MAIN , ANGLE_ROTATION_MAIN)
        #newranges = self.convertXYtoLAserRanges(transformedXY1, self.laser_tmp1)
        

        #Merge
        self.laser_merged.ranges = self.mergeLasers(self.laser_tmp1 , self.laser_tmp2)#laser_aux)
        self.laser_merged.header = self.laser_tmp1.header
        self.laser_merged.header.stamp = rospy.Time.now()
        self.laser_merged = self.make_intensities(self.laser_merged)
        self.laser_merged.angle_increment = ( np.absolute(self.min_ang) + np.absolute(self.max_ang) ) / STEPS
        self.laser_merged_publisher.publish(self.laser_merged)
        self.processing = False

    
    #generate the intensities of the laaser
    def make_intensities(self,laser):
        for i, element in enumerate(laser.ranges):
            if math.isinf(element):
                laser.intensities[i] = 0
        return laser

    def pubStatus(self):
        print(self.laser_tmp1.ranges)

    # replace ranges on laser message for new_ranges_array
    def replaceLaserRanges(self, new_ranges, laser):
        new_laser_ranges = np.zeros(len(laser.ranges)) 
        for i , element in enumerate(new_ranges):
            #print(laser.ranges[i],element,i)
            new_laser_ranges[i] = element
        laser.ranges = new_laser_ranges
        return laser

    # put ranges of laser into a new array
    def mergeLaserRanges(self, destinationRanges, laser_source):
        step = len(destinationRanges) / len(laser_source.ranges)
        for i, element in enumerate(laser_source.ranges):
            destinationRanges[int(i * step)] = element

        return destinationRanges

    # put ranges of laser into a new array out of phase
    def mergeLaserRangesAux(self, destinationRanges, laser_source):
        step = len(destinationRanges) / len(laser_source.ranges)
        for i, element in enumerate(laser_source.ranges):
            #print("stepaux ",step ,"index", int(i * step)-1,"value", element)
            destinationRanges[int(i * step)-1] = element
        return destinationRanges
    
    def convertPointsToLaserRanges(self, points, laser_ref):
        new_laser_ranges = np.ones(len(laser_ref.ranges)) * np.inf
        auxiliarly_array = np.zeros((len(points),2))
        distance_angles = []       
        for i, point in enumerate (points):
            if not np.isinf(point[0]):
                r = np.sqrt((point[0] * point[0]) + (point[1] * point[1])) # r
                auxiliarly_array[i][0] = r
                auxiliarly_array[i][1] = np.arctan2(point[0] , point[1]) # angle
                if r > RANGE_MIN and r < RANGE_MAX:
                    distance_angles.append((auxiliarly_array[i][0] , auxiliarly_array[i][1]))
        serted_list = sorted(distance_angles, key=lambda x: x[1])
        tolerance = 0.008
        angles_array = np.arange(laser_ref.angle_min, laser_ref.angle_max, laser_ref.angle_increment)
        index = 0
        for element in auxiliarly_array:
            for i in range( index, len(laser_ref.ranges)): 
                if angles_array[i-1] - element[1] < tolerance and angles_array[i-1] - element[1] > -tolerance:
                    index = i
                    #print(angles_array[i-1], element[1],angles_array[i-1] - element[1],element[0],i)    
                    new_laser_ranges[i] = element[0]
                    break
        print(len(points),len(new_laser_ranges))
        return new_laser_ranges

    # apply transformation matrix
    def apply_transformation(self, points, translation_x, translation_y, angle):
        transformed_points = np.zeros((len(points),2))
        #pclp = PointCloud()
        #pclp.header.frame_id = "laser"
        for i, point in enumerate(points):
            if not np.isinf(point[0]):
                x = point[0]  * np.cos(angle)  +  point[1] * np.sin(angle) + translation_x #rotation_x + traslation
                y = -point[0] * np.sin(angle)  +  point[1] * np.cos(angle) + translation_y #rotation_y + traslation
                transformed_points[i][0] = x
                transformed_points[i][1] = y
                #pclp.points.append(Point32(y,x,0))        
            else:
                transformed_points[i][0] = float("inf")
                transformed_points[i][1] = float("inf")
        #self.pcl_debug.publish(pclp)
        return transformed_points

    # convert laser ranges  (polar plane) to XY points in cartesian plae(not pointcloud)
    def convertLaserRangesToXY(self,laser):
        converted_array = []
        #pclp = PointCloud()
        #pclp.header.frame_id = "laser"
        for i , theta in enumerate(np.arange(laser.angle_min, laser.angle_max, laser.angle_increment)):
            if not np.isinf(laser.ranges[i]) and laser.ranges[i] > RANGE_MIN and laser.ranges[i] < RANGE_MAX:
                x = laser.ranges[i] * np.sin(theta)
                y = laser.ranges[i] * np.cos(theta)
                converted_array.append((x ,y))
                #pclp.points.append(Point32(y,x,0))
                
        #self.pcl_debug.publish(pclp)
        return np.array(converted_array)

    #convert XY arrar in cartesian plane to polar plane array
    def convertXYtoLAserRanges(self,points,laser):
        converted_array = np.zeros((len(laser.ranges))) 
        #for i , theta in enumerate(np.arange(laser.angle_min, laser.angle_max, laser.angle_increment)):
        for i, laser_range in enumerate(laser.ranges):
            dist_squared = (points[i][0] * points[i][0]) + (points[i][1] * points[i][1])
            r = np.sqrt(dist_squared)
            converted_array[i] = r
            #print(laser.ranges[i], r, i)
        return converted_array


    #merge main and aux lasers
    def mergeLasers(self,laser_main,laser_aux):
        result_merged_ranges = np.zeros(STEPS)
        result_merged_ranges = self.mergeLaserRangesAux(result_merged_ranges , laser_aux)
        result_merged_ranges = self.mergeLaserRanges(result_merged_ranges , laser_main)
        
        return result_merged_ranges
    


def main(args):
    rospy.init_node('laser_merger', anonymous=True)
    laser_merge_obj = laser_merger()
    while not rospy.is_shutdown():
        #laser_merge_obj.pubStatus()
        time.sleep(0.02)

if __name__ == '__main__':
    main(sys.argv)
