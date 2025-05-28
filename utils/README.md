# Utils

## `create-package.bash`

### Usage
`create-package.bash $package_name $module_name [-p] [-h]`
- $package_name -> required, str: name of package
- $module_name -> required, str: name of parent module
- -p -> optional, flag: build python package. Defaults to cpp package.
- -h -> optional, flag: help description.

### Description
- Creates a ros2 python or cpp package_name in the module_name
- Creates missing boilerplate files
    - If CPP build, these are
    - If Python build, these are
- Modifies module_name.interfacing.Dockerfile to copy in package code into container.

<!-- TODO: Populate launch file -->
<!-- TODO: Populate test file for cpp package -->
<!-- TODO: Populate node and core files -->