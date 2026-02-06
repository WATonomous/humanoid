#!/bin/bash

REPO_ROOT=$(git rev-parse --show-toplevel)
MODULES_DIR="$REPO_ROOT/modules"
DOCKERFILE_DIR="$REPO_ROOT/docker"
build_python=false # build package in cpp or python


# Read in keyword arguments
while getopts "ph" opt; do
  case $opt in
    p)
        build_python=true
        ;;
    h)
        echo -e "create-package.bash [-p] [-h] module_name package_name\n"
        echo "Executes \"docker compose up\" module_name and creates package_name with \"ros2 pkg create\"."
        echo "Arguments module_name and package_name are required."
        echo "Use optional -p flag to build a python package. Defaults to a cpp package."
        ;;
    \?)
        echo "Invalid option: -$OPTARG." >&2
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
module_name="$1"
package_name="$2"

if [ -z "$module_name" ] || [ -z "$package_name" ]; then
    echo -e "create-package.bash [-p] [-h] module_name package_name\n"
    echo "Arguments module_name and package_name are required."
    echo "Use optional -p flag to build a python package. Defaults to a cpp package."
  exit 1
fi

# Variables & helper function to copy and edit boilerplate files
PACKAGE_DIR="$REPO_ROOT/autonomy/$module_name/$package_name"
replace_foo_variants_in_file() {
    local file="$1"
    local sed_inplace_flag=(-i)
    replacement_str="$2" # Replace foo's with package_name
    replacement_str_pascal="$(tr '[:lower:]' '[:upper:]' <<< "${replacement_str:0:1}")${replacement_str:1}" # Capitalize first letter for PascalCase (Foo)
    replacement_str_upper="$(tr '[:lower:]' '[:upper:]' <<< "$replacement_str")" # All uppercase (FOO)

    # Detect macOS
    if [[ "$OSTYPE" == "darwin"* ]]; then
        sed_inplace_flag=(-i "")
    fi

    sed "${sed_inplace_flag[@]}" \
        -e "s/Foo/$replacement_str_pascal/g" \
        -e "s/foo/$replacement_str/g" \
        -e "s/FOO/$replacement_str_upper/g" \
        "$file"
}


# Shut down container upon exit
cleanup() {
  if [[ -n "$module_name" ]]; then
    echo "Stopping container for module: $module_name"
    docker compose --profile deploy -f "$MODULES_DIR/docker-compose.$module_name.yaml" down || true
  fi
}
trap cleanup EXIT


# Deploy module_name
docker compose --profile deploy \
    -f $MODULES_DIR/docker-compose.$module_name.yaml \
    up -d $module_name


# Run ros2 pkg create
if [[ "$build_python" != "true" ]]; then # cpp package
    echo "Creating cpp $package_name package."

    docker compose --profile deploy \
    -f $MODULES_DIR/docker-compose.$module_name.yaml \
    exec samples bash -c "
    source /opt/watonomous/setup.bash
    cd ~/ament_ws/src/$module_name
    ros2 pkg create $package_name --build-type ament_cmake --dependencies rclcpp
    cd $package_name
    mkdir launch config test
    touch config/params.yaml
    rm -r include/$package_name
    cd ../..
    colcon build
    "
    
    # Copy in template files into package directory and modify foo's
    cp $REPO_ROOT/utils/boilerplate-files/foo.launch.py $PACKAGE_DIR/launch/$package_name.launch.py
    replace_foo_variants_in_file "$PACKAGE_DIR/launch/$package_name.launch.py" "$package_name"
    cp $REPO_ROOT/utils/boilerplate-files/foo_test.cpp $PACKAGE_DIR/test/${package_name}_test.cpp
    replace_foo_variants_in_file "$PACKAGE_DIR/test/${package_name}_test.cpp" "$package_name"

    cp $REPO_ROOT/utils/boilerplate-files/foo_core.cpp $PACKAGE_DIR/src/${package_name}_core.cpp
    replace_foo_variants_in_file "$PACKAGE_DIR/src/${package_name}_core.cpp" "$package_name"
    cp $REPO_ROOT/utils/boilerplate-files/foo_core.hpp $PACKAGE_DIR/include/${package_name}_core.hpp
    replace_foo_variants_in_file "$PACKAGE_DIR/include/${package_name}_core.hpp" "$package_name"

    cp $REPO_ROOT/utils/boilerplate-files/foo_node.cpp $PACKAGE_DIR/src/${package_name}_node.cpp
    replace_foo_variants_in_file "$PACKAGE_DIR/src/${package_name}_node.cpp" "$package_name"
    cp $REPO_ROOT/utils/boilerplate-files/foo_node.hpp $PACKAGE_DIR/include/${package_name}_node.hpp
    replace_foo_variants_in_file "$PACKAGE_DIR/include/${package_name}_node.hpp" "$package_name"

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
    touch config/params.yaml

    cd ../..
    colcon build
    "

    # Copy in template files into package directory and modify foo's
    cp $REPO_ROOT/utils/boilerplate-files/foo.launch.py $PACKAGE_DIR/launch/$package_name.launch.py
    replace_foo_variants_in_file "$PACKAGE_DIR/launch/$package_name.launch.py" "$package_name"
    cp $REPO_ROOT/utils/boilerplate-files/foo_node.py $PACKAGE_DIR/$package_name/${package_name}_node.py
    replace_foo_variants_in_file "$PACKAGE_DIR/$package_name/${package_name}_node.py" "$package_name"
    cp $REPO_ROOT/utils/boilerplate-files/foo_core.py $PACKAGE_DIR/$package_name/${package_name}_core.py
    replace_foo_variants_in_file "$PACKAGE_DIR/$package_name/${package_name}_core.py" "$package_name"
fi


# Update module_names's Dockerfile to copy in package_name into container
line_to_add="COPY autonomy/$module_name/$package_name $package_name"
marker='# Copy in source code'
dockerfile="$DOCKERFILE_DIR/$module_name/$module_name.Dockerfile"

if ! grep -Fxq "$line_to_add" "$dockerfile"; then # Add only if it does not exist
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
