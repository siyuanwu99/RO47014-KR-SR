#!/usr/bin/env python

# world with 3 tables: table and table_0, and table_1 which is a refrigerated table
# start: milk1, yogurt1 at table
#        honey1 and hagelslag1 at table_1
#        rice1 at table_0

# objective move perishable to table_1: milk1, yogurt1 at table_1 - which pose?
#           move non-perishable to table_0: rice1, honey1, hagelslag - which pose?
#           all movements are possible?


from weakref import ref
import string
import roslib; roslib.load_manifest('rosprolog')
from knowrob_intro.prolog_query import PrologQuery

import rospy
import geometry_msgs.msg
from move_base import MoveBase

from pick_client import PickClient
from place_client import PlaceClient

def query_pose_transform(solution):
    """Convert pose string into pose goal object.
    """
    pose_goal = geometry_msgs.msg.Pose()

    pose_list = list(solution[0].split(", "))
    pose = map(float, pose_list)

    pose_goal.position.x = pose[0]
    pose_goal.position.y = pose[1]
    pose_goal.position.z = pose[2]
    pose_goal.orientation.x = pose[3]
    pose_goal.orientation.y = pose[4]
    pose_goal.orientation.z = pose[5]
    pose_goal.orientation.w = pose[6]

    return pose_goal 

def query_pose_place_transform(solution):
    """Convert pose string into pose goal object.
    """
    point_goal = geometry_msgs.msg.PointStamped()

    pose_list = list(solution[0].split(", "))
    pose = map(float, pose_list)

    point_goal.point.x = pose[0]
    point_goal.point.y = pose[1]
    point_goal.point.z = pose[2]

    return point_goal 

def check_graspable(product):
        # mass of the product
        string_query = "triple(pap:'" + str(product) +"', soma:'hasMassAttribute', X)."
        query = pq.prolog_query(string_query)
        mass_indiv = pq.get_all_solutions(query, False)
        string_query = "triple(pap:'" + str(mass_indiv[0]) +"', soma:'hasMassValue', X)."
        query = pq.prolog_query(string_query)
        # print("Mass value of product {}: ".format(product))
        mass_value = pq.get_all_solutions(query, False)
        # joint effort limit
        string_query = "instance_of(X, soma:'Gripper')."
        query = pq.prolog_query(string_query)
        #print("Available gripper:")
        gripper = pq.get_all_solutions(query, False)

        string_query = "triple(pap:'" + str(gripper[0]) +"_joint', soma:'hasJointLimit', X)."
        query = pq.prolog_query(string_query)
        joint_limit_ind = pq.get_all_solutions(query, False)

        string_query = "triple(pap:'" + str(joint_limit_ind[0]) +"', soma:'hasJointEffortLimit', X)."
        query = pq.prolog_query(string_query)
        #print("Joint effort limit {}: ".format(joint_limit_ind[0]))
        joint_limit_value = pq.get_all_solutions(query, False)

        # if mass  > joint limit, object not graspable
        if float(mass_value[0]) > float(joint_limit_value[0]):
            print(("ERROR product {} not graspable").format(product))
            return False
        else:
            return True



rospy.init_node("example_task")
start = rospy.get_rostime()
pq = PrologQuery() # create a prolog query class instance
print("Cleaning database, this may take 30 s .....")
pq.prolog_query("kb_unproject(_).") # clean previous asserts on MongoDB
# load example.owl in the knowrob database to access its content
query = pq.prolog_query("load_owl('package://knowrob_intro/owl/pick_place.owl', [namespace(pap, 'http://www.airlab.org/tiago/pick-and-place#')]).")

# initialize variables
move_base = MoveBase()


print("\n------------------PART 1: Exploring tables ---------------------------------------------\n")

# Is there any refrigerated table?
query = pq.prolog_query("instance_of(X, pap:'RefrigeratedTable').")
print("Refrigerated table name: ")

refrig_table = pq.get_all_solutions(query)

#  Where is the refrigerated table?
string_query = "triple(pap:'"+ str(refrig_table[0]) +"_pose', soma:'hasPositionData', Y)."
query = pq.prolog_query(string_query)
print("Refrigerated tables: ")
pose_str = pq.get_all_solutions(query)
pose_goal = query_pose_transform(pose_str)

# Moving to table 1
# move_base.run(pose_goal)
print("\n----------------Robot at {}-----------------------------".format(refrig_table[0]))

# Get non-refrigerated tables
query = pq.prolog_query("instance_of(X, soma:'Table'), not(instance_of(X, pap:'RefrigeratedTable')).")
print("\nNon-refrigerated tables: ")
tables = pq.get_all_solutions(query)
for table in tables:
    # Where is table?
    string_query = "triple(pap:'"+ str(table) +"_pose', soma:'hasPositionData', Y)."
    query = pq.prolog_query(string_query)
    print("Table {} pose: ".format(table))
    pose_str = pq.get_all_solutions(query)
    pose_goal = query_pose_transform(pose_str)

    # Moving to table
    #move_base.run(pose_goal)
    print("\n----------------Robot at {}-----------------------------".format(table))

    # Which products are in this table?
    string_query = "triple(X, dul:'hasLocation', pap:'"+ str(table) +"')."
    query = pq.prolog_query(string_query)
    print("\nProducts: ")
    products_t = pq.get_all_solutions(query)
print("\nAll tables explored")


# print("\n------------------PART 2: Exploring products ---------------------------------------------\n")
products_to_move ={} # product:destination table

# Which products are in table1?
string_query = "triple(X, dul:'hasLocation', pap:'"+ str(refrig_table[0]) +"')."
query = pq.prolog_query(string_query)
print("\nProducts in table 1: ")
products_tb1 = pq.get_all_solutions(query)

# Which products are in table or table 0?
string_query = "triple(X, dul:'hasLocation', pap:'"+ str(tables[0]) +"'); triple(X, dul:'hasLocation', pap:'"+ str(tables[1]) +"')."
query = pq.prolog_query(string_query)
print("\nProducts in non-refrigerated table: ")
products_tb = pq.get_all_solutions(query)


# Are they in the correct pose? 
# a/perishable in table
print("---------------------Checking perishables ----------")
for prd_tb1 in products_tb1:
    string_query = "instance_of(pap:'" + str(prd_tb1) +"', pap:'Perishable')."
    query = pq.prolog_query(string_query)
    print(("{} is Perishable: ").format(prd_tb1))
    perishable = pq.get_all_solutions(query)

    if perishable[0] == 'false': # products not in the correct table
        # the product needs to move, check if object is graspable
        grasp = check_graspable(prd_tb1)
        if grasp == True:
            # store product to move    
            products_to_move[prd_tb1] = tables[0] # move non-perishable product to the first non refrigerated table

# b/ non-perishable table or table 0
print("--------------------Checking not perishables ----------")
for prd_tb in products_tb:
    string_query = "not(instance_of(pap:'" + str(prd_tb) +"', pap:'Perishable'))."
    query = pq.prolog_query(string_query)
    print(("{} is not Perishable: ").format(prd_tb))
    not_perishable = pq.get_all_solutions(query)

    if not_perishable[0] == 'false': # products not in the correct table
        # the product needs to move, check if object is graspable
        grasp = check_graspable(prd_tb)
        if grasp == True:
            # store product to move    
            products_to_move[prd_tb] = refrig_table[0] # move perishable product to the first refrigerated table

for k,v in products_to_move.items():
    print("Moving product {} to {}".format(k, v))


print("\n--------------------------------- Moving products---------------------------------\n")

for product, destination in products_to_move.items():
    # move to product origin table
    string_query = "triple(pap:'" + str(product)+ "', dul:'hasLocation', Y)."
    query = pq.prolog_query(string_query)
    table_origin = pq.get_all_solutions(query, False)

    string_query = "triple(pap:'"+ str(table_origin[0]) +"_pose', soma:'hasPositionData', Y)."
    query = pq.prolog_query(string_query)
    pose_str = pq.get_all_solutions(query, False)
    pose_goal = query_pose_transform(pose_str)
    move_base.run(pose_goal)

    # get aruco id
    string_query = "triple(pap:'"+ str(product) +"', pap:'has_aruco_ID', Y)."
    query = pq.prolog_query(string_query)
    print("Aruco ID: ")
    id = pq.get_all_solutions(query)
    id = map(int, id)

    # pick
    pick_client = PickClient("right")
    pick_client.run(aruco_id = id[0])

    # get destination pose
    string_query = "triple(pap:'"+ str(destination) +"_pose', soma:'hasPositionData', Y)."
    query = pq.prolog_query(string_query)
    pose_str = pq.get_all_solutions(query, False)
    pose_goal = query_pose_transform(pose_str)

    # move
    move_base.run(pose_goal)

    # place
    # get table_pose_goal
    string_query = "triple(pap:'"+ str(product) +"_table_pose', soma:'hasPositionData', Y)."
    query = pq.prolog_query(string_query)
    pose_str = pq.get_all_solutions(query)
    point_goal = query_pose_place_transform(pose_str)

    # placing
    place_client = PlaceClient("right")
    point_goal.header.frame_id = "base_footprint"
    place_client.run(position=point_goal)
    print("Product {} moved to {}".format(product, destination))
    

    # updating pose
    now = rospy.get_rostime()
    string_query = "kb_project(holds(pap:'" + str(product)+ "', dul:'hasLocation', pap:'" + table_origin[0] + "') during [" + str(start.secs) + ", " + str(now.secs) + "])."
    query = pq.prolog_query(string_query)

    string_query = "kb_project(holds(pap:'" + str(product)+ "', dul:'hasLocation', pap:'" +  table_origin[0] + "') until " + str(now.secs) + ")."
    query = pq.prolog_query(string_query)

    string_query = "kb_project(holds(pap:'" + str(product)+ "', dul:'hasLocation', pap:'" + destination + "') since " + str(now.secs) + ")."
    query = pq.prolog_query(string_query)

    # checking pose
    string_query = "kb_call(holds(pap:'" + str(product)+ "', dul:'hasLocation', pap:'" + table_origin[0] + "') during [" + str(start.secs) + ", " + str(now.secs) + "])."
    query = pq.prolog_query(string_query)
    sol = pq.get_all_solutions(query)
    print("{} was at {} during [{}, {}]: {}".format(product, table_origin[0], start.secs, now.secs, sol[0]))

    string_query = "kb_call(holds(pap:'" + str(product)+ "', dul:'hasLocation', pap:'" + table_origin[0] + "') since " + str(now.secs) + ")."
    query = pq.prolog_query(string_query)
    sol = pq.get_all_solutions(query)
    print("{} is at {} since {}: {}".format(product, table_origin[0], now.secs, sol[0]))

    string_query = "kb_call(holds(pap:'" + str(product)+ "', dul:'hasLocation', pap:'" + destination + "') since " + str(now.secs) + ")."
    query = pq.prolog_query(string_query)
    sol = pq.get_all_solutions(query)
    print("{} is at {} since {}: {}".format(product, destination, now.secs, sol[0]))
    #break # only one product testing




