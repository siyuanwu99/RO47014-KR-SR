# RO47014-KR-SR
Course project for RO47014 Knowledge Representation and Symbolic Reasoning

```
git clone <this repo>
cd RO47014-KR-SR
git submodule update --init --recursive
```


# UPDATE 08/03/2022

To load AH_store models, copy `./setup.sh` to your workspace

source this file in your singularity image

# Understanding the environment

To launch the project environment, use the command below after sourcing all the files:
```
roslaunch retail_store_simulation project.launch
```
In the projecy.world file, the environment mainly contains two parts: 
* **Model part**: from line 16 to 4791. It contains 11 models including `ground_plane`, `AH_cooling`, `AH_bonus`, `AH_shelf`, `room_square`, `AH_bonus_clone`, `AH_cooling_clone`, `table`, `basket_with_aruco`, `AH_shelf_filled` and `AH_hagelslag_aruco_15`. It contains parameters including mass, inertia, relative pose, visual, collision.
* **State part**: from line 4793 to 4992. This part contains the specific parameters (global pose, scale, velocity, acceleration, wrench) of different models. In addition to the 11 models above, it also contain the parameters of the sun. 

There are some models we should pay special attention to:
* `AH_shelf_filled`: In line 4869, the global pose is (-2.99822 0.021911 0 0 0 -1.5708). It has link to its model `board`.
* `AH_shelf_filled::board`: This is the lowest level of the shlef, but it is above the ground plane. In line 4878, the global pose is (-2.498 0.021911 0.25 0 0 -1.5708), the height is 0.25. In line 1118, the relative pose is (0 0.5 0.25 0 -0 0). In the gazebo simulation window, the pose is (x=0, y=0.5, z=0.25, roll=pitch=yaw=0). It has links to 5 hagelslag `AH_hagelslag_aruco_10-14`. 
* `AH_hagelslag_aruco_10::link_0`: In line 4881, the global pose is (-2.498 -0.0892 0.1635 0 -0 -0.000739). In line 1132, the relative pose is (0.111111 0 0 0 -0 1.57). In the gazebo simulation window, the pose is (x=0.1111, y=0.0, z=-0.0865, yaw=1.57) (it is not the same with line 1132, kind of confused). 
* `table`: In line 964 and line 4963, the global pose is (2 3 0 0 -0 0), the scale is (1 1 1).
* `table::AH_hagelslag_aruco_16::link_0`: In line 4973, the global pose is (2 2.6 0.855 0 -0 -1.5708). In line 892, the relative pose to the table is (0 -0.4 0.855 0 -0 -1.5708).
* `AH_hagelslag_aruco_15`:
* `basket_with_aruco`: