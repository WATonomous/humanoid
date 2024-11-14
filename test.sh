#!/bin/bash

# set -e
if [[ -z "${DEV_DEPENDENCIES_MOUNTED}" ]]; then
    export DEV_DEPENDENCIES_MOUNTED=true
    export TEMP_DEPENDENCES_DIR="/tmp/deps"
fi

# run containers to mount and expose data to vscode
if [[ $1 == "setup" ]]; then
    echo starting setup

    mkdir -p ${TEMP_DEPENDENCES_DIR}
    
    docker compose --profile develop -f /home/t95zhou/repos/wato_asd_training/modules/docker-compose.robot.yaml up --remove-orphans --detach 
    docker cp watod_$(whoami)-robot_dev-1:/opt/ros $TEMP_DEPENDENCES_DIR
    cd $TEMP_DEPENDENCES_DIR/ros/humble
    . ./setup.bash
    docker compose -f /home/t95zhou/repos/wato_asd_training/modules/docker-compose.robot.yaml down

else
    echo Invalid command. Available commands: setup
fi
