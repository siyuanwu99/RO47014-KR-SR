## Grocery Store utilities

This repo contains utility servers for robotic manipulation within a grocery store. The following servers are available:

- *collision_obstacles_server*: This server provides two easy to use services to manage the creation and deletion of collision obstacles in the moveit planningscene. The server uses prior knowledge of the geometry of the object, and placement of the aruco marker. To add new object types, edit the function `register_collision_object`. The services are:
  - **add_collision_object**: Adds a collision object by detected aruco pose, additionally the object type must be provided in the request so the service knows the geometry of the object w.r.t the marker.
  - **remove_collision_object**: Removes a collision object, by requested object id.

- *grasp_pose_server*: This server provides a service called `get_grasp_pose` that returns a predefined grasp pose of a certain object type. The grasp pose is specified in the frame of the detected aruco marker. Note that the aruco marker frame has to exist for the service to work. It is therefore recommended to first register the collision object using the `add_collision_object` service, which will broadcast the aruco marker frame.

- *grocery_list_server*: This server is provides a number of services to manage a grocery list.
  - **set_list_server**: Not Implemented. Currently the inventory is hard coded, and the list is loaded from `my_grocery.txt` during server startup.
  - **list_info_server**: Get info about grocery list. Currently only returns the length of the list.
  - **get_product_server**: Get info about the product at the top of the grocery list.
  - **remove_product_server**: Remove product at the top of the grocery list.


> Further notes: The script `grocery_list_server.py` so far keeps the same order as in the `.txt`. The inventory variable in the script `grocery_list_server.py` contains a dictionary with product names and locations in front of the shelves from where it is possible to pick them. This will be moved into a better handled ontology of the store in the future.
 
### Usage

To run the services use the launch file:

`roslaunch grocery_store_utils grocery_utils.launch`

### Future Goals

- [ ] Create Object Manager independent of moveit's collision object server. This module should:
  - [ ] Contain a 'database' of information about objects often used in grocery stores (products, basket, etc.). Should at least contain object's geometry, relative position of aruco marker on object, predefined grasp pose of the object.
  - [ ] Good interface with moveit's collision object server, to create update and delete the managed objects. But it must be possible to run this module without moveit, so the interface should be optional.
  - [ ] Expose services to easily register new objects (initially by aruco marker position and known object type)
  - [ ] Expose services to retrieve grasp pose (initially predefined)
  - [ ] Track Objects over time (again initially using aruco markers) and publish to the TF server
  - [ ] (future) Register new objects through 3D detection algorithms (Deep NN's) 
  - [ ] (future) Get grasp pose through grasp pose detection networks (Deep NN's) 
