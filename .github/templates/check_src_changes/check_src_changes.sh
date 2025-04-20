#!/bin/bash
set -e

################# Create a space delimited list of modified modules #################
# Outputs a list of modified modules by comparing changes between main and current commit
# References previous GitHub workflow steps

# Controller
if [ $CONTROLLER_CHANGED == 'true' ]; then
    echo "Detected controller changes"
    MODIFIED_MODULES+="controller "
fi

# Interfacing
if [ $INTERFACING_CHANGED == 'true' ]; then
    echo "Detected interfacing changes"
    MODIFIED_MODULES+="interfacing "
fi

# Perception
if [ $PERCEPTION_CHANGED == 'true' ]; then
    echo "Detected perception changes"
    MODIFIED_MODULES+="perception "
fi

# Samples
if [ $SAMPLES_CHANGED == 'true' ]; then
    echo "Detected samples changes"
    MODIFIED_MODULES+="samples "
fi

# Simulation
if [ $SIMULATION_CHANGED == 'true' ]; then
    echo "Detected simulation changes"
    MODIFIED_MODULES+="simulation "
fi
# Embedded
if [ $EMBEDDED_CHANGED == 'true' ]; then
    echo "::notice:: Detected infrastructure changes"
    MODIFIED_MODULES+="embedded"
fi 

# Infrastructure
if [ $INFRASTRUCTURE_CHANGED == 'true' ]; then
    echo "::notice:: Detected infrastructure changes"
    MODIFIED_MODULES="infrastructure"
else
    echo "::notice:: MODIFIED_MODULES are $MODIFIED_MODULES" 
fi



# Output lis
echo "modified_modules=$MODIFIED_MODULES" >> $GITHUB_OUTPUT