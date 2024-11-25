#!/bin/bash

set -e
if [[ -z "${DEV_DEPENDENCIES_MOUNTED}" ]]; then
    export DEV_DEPENDENCIES_MOUNTED=true
    export TEMP_DEPENDENCES_DIR="/tmp/deps"
fi

# run containers to mount and expose data to vscode
if [[ $1 == "setup" ]]; then
    echo starting setup

    mkdir -p ${TEMP_DEPENDENCES_DIR}
    
    docker compose --profile develop -f /home/eddyzhou/wato_asd_training/modules/docker-compose.robot.yaml up --remove-orphans --detach 
    docker cp watod_$(whoami)-robot_dev-1:/opt/ros $TEMP_DEPENDENCES_DIR
    cd $TEMP_DEPENDENCES_DIR/ros/humble
    . ./setup.bash
    docker compose -f /home/eddyzhou/wato_asd_training/modules/docker-compose.robot.yaml down

else
    echo Invalid command. Available commands: setup
fi

# Create the .vscode directory if it doesn't exist
# Define the .vscode directory path
VSCODE_DIR="/home/eddyzhou/wato_asd_training/.vscode"

# Create the .vscode directory if it doesn't exist
mkdir -p "$VSCODE_DIR"

# Check if the directory exists
if [ -d "$VSCODE_DIR" ]; then
    # Delete c_cpp_properties.json if it exists
    if [ -f "$VSCODE_DIR/c_cpp_properties.json" ]; then
        rm "$VSCODE_DIR/c_cpp_properties.json"
        echo "Deleted $VSCODE_DIR/c_cpp_properties.json"
    else
        echo "$VSCODE_DIR/c_cpp_properties.json does not exist"
    fi

    # Delete settings.json if it exists
    if [ -f "$VSCODE_DIR/settings.json" ]; then
        rm "$VSCODE_DIR/settings.json"
        echo "Deleted $VSCODE_DIR/settings.json"
    else
        echo "$VSCODE_DIR/settings.json does not exist"
    fi
else
    echo "Directory $VSCODE_DIR does not exist"
fi

# Write to c_cpp_properties.json
cat << EOF > /home/eddyzhou/wato_asd_training/.vscode/c_cpp_properties.json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "/tmp/deps/**",
                "\${workspaceFolder}/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c17",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
EOF

# Write to settings.json
cat << EOF > /home/eddyzhou/wato_asd_training/.vscode/settings.json
{
    "cmake.ignoreCMakeListsMissing": true,
    
    "python.autoComplete.extraPaths": [
        "/tmp/deps/ros/humble/local/lib/python3.10/dist-packages"
    ],
    "python.analysis.extraPaths": [
        "/tmp/deps/ros/humble/local/lib/python3.10/dist-packages"
    ],
    "python.analysis.autoSearchPaths": true
}
EOF

echo "Configuration files created successfully in .vscode/"
touch /home/eddyzhou/wato_asd_training/.vscode/c_cpp_properties.json
touch /home/eddyzhou/wato_asd_training/.vscode/settings.json