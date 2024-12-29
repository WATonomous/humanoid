#!/bin/bash

SERVICE_NAME=$1

echo "Starting dependency mount"

TEMP_DEPENDENCES_DIR="/tmp/deps"
DEP_MOUNT_CONTAINER_NAME=dep-mount-temp

mkdir -p "${TEMP_DEPENDENCES_DIR}"

# Dynamically create a Docker Compose service
run_compose run --rm --detach --name "${DEP_MOUNT_CONTAINER_NAME}" "${SERVICE_NAME}" tail -F anything

# Copy dependencies from the service's container to a temporary directory
docker cp "${DEP_MOUNT_CONTAINER_NAME}:/opt/ros" "${TEMP_DEPENDENCES_DIR}"

# Source ROS environment
cd "${TEMP_DEPENDENCES_DIR}/ros/humble"
. ./setup.bash

# Stop the service
docker stop "${DEP_MOUNT_CONTAINER_NAME}"

# Create the .vscode directory if it doesn't exist
# Define the .vscode directory path
VSCODE_DIR="$MONO_DIR/.vscode"

# Create the .vscode directory if it doesn't exist
mkdir -p "$VSCODE_DIR"

# Write to c_cpp_properties.json
cat << EOF > $VSCODE_DIR/c_cpp_properties.json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "\${workspaceFolder}/**",
                "/tmp/deps/**"
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
cat << EOF > $VSCODE_DIR/settings.json
{
    "python.analysis.extraPaths": [
        "/tmp/deps/ros/humble/local/lib/python3.10/dist-packages"
    ],
    "python.autoComplete.extraPaths": [
        "/tmp/deps/ros/humble/local/lib/python3.10/dist-packages"
    ],
    "[python]": {
        "editor.formatOnSave": true,
        "editor.defaultFormatter": "charliermarsh.ruff"
    },
    "python.analysis.autoSearchPaths": true,
}
EOF

# Write to settings.json
cat << EOF > $VSCODE_DIR/extensions.json
{
    "recommendations": [
        "ms-iot.vscode-ros",
        "charliermarsh.ruff"
    ],
}
EOF

echo "Configuration files created successfully in .vscode/"

# Display the message with yellow text and borders
echo -e "\033[1;33m##############################################\033[0m"
echo -e "\033[1;33m#                                            #\033[0m"
echo -e "\033[1;33m#   VScode Dev Environment Set Up!           #\033[0m"
echo -e "\033[1;33m#   Type CMD/CTRL + SHIFT + P > Reload       #\033[0m"
echo -e "\033[1;33m#   Window to activate IntelliSense          #\033[0m"
echo -e "\033[1;33m#                                            #\033[0m"
echo -e "\033[1;33m##############################################\033[0m"
