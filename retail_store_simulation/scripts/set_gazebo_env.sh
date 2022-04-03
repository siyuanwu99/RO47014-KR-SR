#!/bin/bash

export GAZEBO_RESOURCE_PATH=$(rospack find retail_store_simulation):$GAZEBO_RESOURCE_PATH
# export GAZEBO_MODEL_PATH=$(rospack find retail_store_simulation)/models:$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=$(rospack find retail_store_simulation)/models/AH_hagelslag:$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=$(rospack find retail_store_simulation)/models/AH_thee:$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=$(rospack find retail_store_simulation)/models/AH_store:$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=$(rospack find retail_store_simulation)/models/legacy_models:$GAZEBO_MODEL_PATH
