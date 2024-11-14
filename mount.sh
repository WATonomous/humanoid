#!/bin/bash

set -e
echo run this script with source mount.sh setup
if [[ ${BASH_SOURCE[0]} != ${0} ]]; then
    # run containers to mount and expose data to vscode
    if [[ $1 == "setup" ]]; then
        
        echo starting setup

        TEMP_DEPENDENCES_DIR="/tmp/deps"
        SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
        DEP_MOUNT_IMAGE_NAME=ros:humble
        DEP_MOUNT_CONTAINER_NAME=dep-mount

        mkdir -p ${TEMP_DEPENDENCES_DIR}
        
        echo mounting dependencies
        docker run --rm --detach --name $DEP_MOUNT_CONTAINER_NAME $DEP_MOUNT_IMAGE_NAME sleep infinity > /dev/null
        docker cp $DEP_MOUNT_CONTAINER_NAME:/opt/ros $TEMP_DEPENDENCES_DIR
        cd $TEMP_DEPENDENCES_DIR/ros/humble
        . ./setup.bash
        docker stop $DEP_MOUNT_CONTAINER_NAME > /dev/null
        cd $SCRIPT_DIR
    else
        echo Invalid command. Available commands: setup
    fi
fi
