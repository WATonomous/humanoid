# Utils

## `create-package.bash`

### Usage
`create-package.bash $package_name $module_name [-p] [-h]`
- $package_name -> required, str: name of package
- $module_name -> required, str: name of parent module
- -p -> optional, flag: build python package. Defaults to a cpp package.
- -h -> optional, flag: help description.

- ⚠️ The `module_name` must exist beforehand for the script to work properly.

### Description
- Creates a ros2 python or cpp `package_name` in the `module_name`
- Creates missing boilerplate files
    - If CPP build, these are:
        - launch/`package_name`.launch.py
        - config/params.yaml
        - src/`package_name`_node.cpp
        - include/`package_name`_node.hpp
        - src/`package_name`_core.cpp
        - include/`package_name`_core.hpp
        - test/`package_name`_test.cpp
    - If Python build, these are:
        - launch/`package_name`.launch.py
        - config/params.yaml
        - `package_name`/`package_name`_node.py
        - `package_name`/`package_name`_core.py
- Modifies module_name.interfacing.Dockerfile to copy in package code into container.