#!/bin/bash

REPO_ROOT=$(git rev-parse --show-toplevel)
MODULES_DIR="$REPO_ROOT/modules"
DOCKERFILE_DIR="$REPO_ROOT/docker"
build_python=false # build package in cpp or python

# Keyword arguments
while getopts "ph" opt; do
  case $opt in
    p)
        build_python=true
        ;;
    h)
        echo "create-package.bash [-p] [-h] package_name module_name"
        echo "Executes \"docker compose up\" module_name and creates package_name with \"ros2 pkg create\""
        echo "Required: package_name, module_name"
        echo "Use optional -p flag to build a python package. Defaults to a cpp package."
        ;;
    \?)
        echo "Invalid option: -$OPTARG" >&2
        exit 1
        ;;
    :)
        echo "Option -$OPTARG requires an argument." >&2
        exit 1
        ;;
  esac
done

shift $((OPTIND -1))

# Positional arguments
package_name="$1"
module_name="$2"

# Deploy module_name
docker compose --profile deploy \
    -f $MODULES_DIR/docker-compose.$module_name.yaml \
    up -d $module_name

# Run ros2 pkg create
if [!build_python]; then
    echo "Creating cpp $package_name package."

    docker compose --profile deploy \
    -f $MODULES_DIR/docker-compose.$module_name.yaml \
    exec interfacing bash -c "
    source /opt/watonomous/setup.bash
    cd ~/ament_ws/src/$module_name
    ros2 pkg create $package_name --build-type ament_cmake --dependencies rclcpp
    cd $package_name
    mkdir launch config test
    touch launch/$package_name.launch.py config/params.yaml src/${package_name}_node.cpp src/${package_name}_core.cpp include/${package_name}_node.hpp include/${package_name}_core.hpp test/src/${package_name}_test.cpp
    rm src/$package_name.cpp include/$package_name.hpp
    
    cd ../..
    colcon build
    "

else # python package
    echo "Creating python $package_name package."

    docker compose --profile deploy \
    -f $MODULES_DIR/docker-compose.$module_name.yaml \
    exec interfacing bash -c "
    source /opt/watonomous/setup.bash
    cd ~/ament_ws/src/$module_name
    ros2 pkg create $package_name --build-type ament_python --dependencies rclpy
    cd $package_name
    mkdir launch config
    touch launch/$package_name.launch.py config/params.yaml $package_name/${package_name}_node.py $package_name/${package_name}_core.py

    cd ../..
    colcon build
    "
fi

# Update module_names's Dockerfile to copy in package_name into container
# Add only if it does not exist
line_to_add="COPY $MODULES_DIR/$module_name/$package_name $package_name"
marker='# Copy in source code'
dockerfile="$DOCKERFILE_DIR/$module_name/$module_name.Dockerfile"

if ! grep -Fxq "$line_to_add" "$dockerfile"; then
    echo "Adding COPY line to $dockerfile"

    # Check if on macOS
    if [[ "$OSTYPE" == "darwin"* ]]; then
        # Use `|` as delimiter to avoid issues with `/`
        sed -i '' "/^${marker}[[:space:]]*$/a\\
$line_to_add
" "$dockerfile"
    else
        sed -i "/^${marker}[[:space:]]*$/a $line_to_add" "$dockerfile"
    fi
fi