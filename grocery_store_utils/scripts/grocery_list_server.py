#!/usr/bin/env python

#######################
#
# Script to hande the list of products to be used in the BT during the demo Xplore 2021
#
# The script reads the list from a file my_grocery.txt and builds the order list always fetching the closest product.
# If a list is not valid the user is required to adjust it.
# The inventory variable contains a dictionary with product names and locations in front of the shelves from where it is possible to pick them
#
# Easy test this script: rosservice call /list_info_server "{}" 
#
# Author: Corrado Pezzato, TU Delft, AIRLab. 2021.05.25
#
#######################

# import rospy
import os, sys
import numpy as np
import rospy
from grocery_store_utils.srv import *


def set_inventory():
    product1 = {"name": "hageslag",           "aruco_id": 4,   "object_type": "carton", "base_loc_x": 2.44,  "base_loc_y": 0.38,  "base_theta": -0.05}
    product2 = {"name": "tomataten blockjes", "aruco_id": 2,   "object_type": "carton", "base_loc_x": 2.62,  "base_loc_y": -0.52, "base_theta": -0.13}
    product3 = {"name": "banana",             "aruco_id": 1,   "object_type": "carton", "base_loc_x": -0.35, "base_loc_y": -0.19, "base_theta": 3.14}
    product4 = {"name": "melk1",              "aruco_id": 0,   "object_type": "carton", "base_loc_x": 2.68,  "base_loc_y": 0.70,  "base_theta": 0.0}
    product5 = {"name": "melk2",              "aruco_id": 10,  "object_type": "carton", "base_loc_x": 2.68,  "base_loc_y": 0.50,  "base_theta": 0.0}
    product6 = {"name": "box1",               "aruco_id": 444, "object_type": "carton", "base_loc_x": 1.28,  "base_loc_y": 0.03,  "base_theta": 0}
    product7 = {"name": "box2",               "aruco_id": 582, "object_type": "carton", "base_loc_x": -1.12, "base_loc_y": 1.2,   "base_theta": 1.57}
    product8 = {"name": "hageslag_sim",       "aruco_id": 111, "object_type": "carton", "base_loc_x": -2.30, "base_loc_y": -0.05, "base_theta": -3.14}

    inventory = [product1, product2, product3, product4, product5, product6, product7, product8]
    return inventory

def list_info_callback(req):
    resp = listInfoResponse()
    resp.listLen = len(grocery_list)
    return resp   
    
def get_product_callback(req):
    resp = getProductResponse()

    if len(grocery_list) > 0:
        # Get inventory index and retrieve product info from first element of grocery list
        itemToFetch = grocery_list[0]
        for index in range(len(inventory)):
            if itemToFetch == inventory[index]["name"]:
                inventory_index = index
                break
        
        currProduct = inventory[inventory_index]
        # Build response
        resp.arucoId = int(currProduct["aruco_id"])
        resp.base_pose.x = float(currProduct["base_loc_x"])
        resp.base_pose.y = float(currProduct["base_loc_y"])
        resp.base_pose.theta = float(currProduct["base_theta"])
        print("Product to be fetched has been added to the response")
        resp.flag = 1  # flag ok
        return resp
    else:
        rospy.loginfo("The list is empty, no prduct to fetch")
        resp.flag = -1  # error flag
        return resp

def remove_product_callback(req):
    resp = removeProductResponse()

    if len(grocery_list) > 0:
        grocery_list.pop(0)
        print("Remaining grocery list", grocery_list)
        resp.flag = 1  # flag ok
        return resp
    else:
        resp.flag = -1  # error flag
        rospy.loginfo("The list is empty, no prduct to remove from it")
        return resp

def set_list_callback(req):
    resp = setListResponse()
    # TODO: Allow setting of grocery list via service

    resp.flag = 1  # flag ok
    return resp

def euler_to_quaternion(roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

def quaternion_to_euler(x, y, z, w):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    roll = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    pitch = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    yaw = np.degrees(np.arctan2(t3, t4))

    return [roll, pitch, yaw]


if __name__ == "__main__":
    
    ## Initialization
    rospy.init_node('grocery_server')

    # Initialize services for BT
    rospy.Service('list_info_server', listInfo, list_info_callback)
    rospy.Service('get_product_server', getProduct, get_product_callback)
    rospy.Service('remove_product_server', removeProduct, remove_product_callback)
    rospy.Service('set_list_server', setList, set_list_callback)


    # Available products
    inventory = set_inventory()
    product_names = []
    for item in inventory:  
        product_names.append(item["name"])
    
    print('\nHey there! I am processing your order... \n')
    
    # Read file 
    grocery_file = open(os.path.join(sys.path[0], "my_grocery.txt"), "r")
    grocery_file_lines = grocery_file.readlines()
    grocery_file.close()
    grocery_list = []

    # Take only products from the file, they start with "."
    for line in grocery_file_lines:
        if line[0] == '.':
            # Remove the . and the \n
            grocery_list.append(line[1:-1])
        
    # Check if every product is in the inventory, if not ask to fix the list
    valid_list = 1
    for item in grocery_list:
        if item in product_names:
            pass
        else:
            print("The item", item, "is not available")
            valid_list = 0

    if valid_list:
        print("The grocery list is:", grocery_list)
        # Order the list by distance if necessary        
    else:
        print("\nPlease insert a valid order in my_grocery.txt")
        sys.exit(1)

    rospy.loginfo("\nReady to handle your order!")
    rospy.spin()

'''
Store positions:
================

Banana: 
  pose: 
    position: 
      x: -0.348866302664
      y: -0.195142426656
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.999778957929
      w: 0.0210246351384
yaw: 3.14

Milk: 
  pose: 
    position: 
      x: 0.892665824729
      y: 1.28807536526
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.674591405341
      w: 0.738191327394
yaw: 1.48

Hageslag: 
  pose: 
    position: 
      x: 2.4417163358
      y: 0.388346324744
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.0271413055269
      w: 0.99963160691
yaw: -0.05

Tomaten: 
  pose: 
    position: 
      x: 2.62128121872
      y: -0.524313652068
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.0654516529906
      w: 0.997855741638
yaw: -0.13

applesientje: 
  pose: 
    position: 
      x: 2.51829302057
      y: 0.124207242945
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.0543893899363
      w: 0.998519801638
yaw: -0.1

'''


'''
Locations for retail store simulation

waypoints:
  wp_table_1:
    position:
      x: -1.12
      y: 1.20
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.707
      w: 0.707
  wp_table_2:
    position:
      x: 0.80
      y: 1.20
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.707
      w: 0.707
  wp_table_3:
    position:
      x: 1.28
      y: -0.03
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0 
      w: 1.0
  wp_cabinet_1:
    position:
      x: 0.48
      y: -1.24
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.707
      w: 0.707
  wp_cabinet_2:
    position:
      x: -0.4356
      y: -1.24
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.707
      w: 0.707

'''
