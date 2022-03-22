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
* **Model part**: from line 16 to 4795. It contains 11 models including `ground_plane`, `AH_cooling`, `AH_bonus`, `AH_shelf`, `room_square`, `AH_bonus_clone`, `AH_cooling_clone`, `table`, `basket_with_aruco`, `AH_shelf_filled` and `AH_hagelslag_aruco_15`
* **State part**: from line 4796 to 4996. This part contains the specific parameters (pose, scale, velocity, acceleration, wrench) of different models. In addition to the 11 models above, it also contain the parameters of the sun. 

There are some models we should pay special attention to:
* `AH_shelf_filled`: 
* `table`:
* `AH_hagelslag_aruco_15`:
* `basket_with_aruco`: