# RO47014-KR-SR: TiAGo: A Sorting Robot for Hagelslag

Course project for RO47014 Knowledge Representation and Symbolic Reasoning.

We developed a robot to help shop staffs find damaged or empty items and place well-preserved items on the shelves. Our robot can achieve

1. Select empty products from all products
2. Discard the empty products in the basket
3. Place the full products on the shelf

<img src="https://user-images.githubusercontent.com/44539400/161447968-e2f5878a-2833-4f85-99ae-13733eb59a07.png" width=500 >

```
git clone <this repo>
cd RO47014-KR-SR
git submodule update --init --recursive
```

make sure you have installed ROSPlan and other necessary modules. Then build from source code.

```
catkin build
```

Then you can launch our environment.

```
roslaunch retail_store_simulation project.launch
```

Start another terminal and launch ROSPlan server
```
roslaunch group13 rosplan_project.launch
```

Start the 3rd terminal and run following script
```
source <your-workspace>/src/retail_store_skills/group13_planning/rosplan_executor.bash
```


## Demo


### Pick

robot pick products from the desk

- Refined grasp position
- Implemented pre/post grasp pose

<img src="https://github.com/edmundwsy/RO47014-KR-SR/blob/main/media/pre_post_grasp.gif" height="300">

### Place

robot place products on the shelf

- Refined place pose
- Implemented pre/post place pose

<img src="https://github.com/edmundwsy/RO47014-KR-SR/blob/main/media/place_right.gif" height="300">

### Discard

robot move to the basket and throw products

<img src="https://github.com/edmundwsy/RO47014-KR-SR/blob/main/media/discard_left.gif" height="300">

### Full demo

speed x8

https://user-images.githubusercontent.com/44539400/161447861-fee4297e-f6f9-449b-9f94-71b063460fd9.mp4



## UPDATE 08/03/2022

To load AH_store models, copy `./setup.sh` to your workspace

source this file in your singularity image

## Understanding the environment

To launch the project environment, use the command below after sourcing all the files:
```
roslaunch retail_store_simulation project.launch
```
In the project.world file, the environment mainly contains two parts: 
* **Model part**: from line 16 to 5078. It contains 14 models including `ground_plane`, `AH_cooling`, `AH_bonus`, `AH_shelf`, `room_square`, `AH_bonus_clone`, `AH_cooling_clone`, `table_1`, `basket_with_aruco`, `AH_shelf_filled`, `AH_hagelslag_aruco_16`, `AH_hagelslag_aruco_17`, `AH_hagelslag_aruco_0`, `AH_hagelslag_aruco_1`. It contains parameters including mass, inertia, relative pose, visual, collision.
* **State part**: from line 5080 to 5290. This part contains the specific parameters (global pose, scale, velocity, acceleration, wrench) of different models. In addition to the 14 models above, it also contain the parameters of the sun. 

There are some models that are closely related to our project:
* `table_1`: In line 988, the global pose is (2 3 0 0 -0 0).
* `basket_with_aruco`: In line 5218, the global pose is (-1.0 2.5 0.605927 3e-05 -0 -1.70074). In line 5219, the scale is set to (2 2 1). In line 994, the mass is set to 2.5.
* `AH_hagelslag_aruco_16`: In line 5248, the initial position is (2.3 2.65 1 0 -0 -1.5708).
* `AH_hagelslag_aruco_0`: In line 5258, the initial position is (2.1 2.65 1 0 -0 -1.5708).
* `AH_hagelslag_aruco_17`: In line 5268, the initial position is (1.9 2.65 1 0 -0 -1.5708).
* `AH_hagelslag_aruco_1`: In line 5278, the initial position is (1.7 2.65 1 0 -0 -1.5708).
