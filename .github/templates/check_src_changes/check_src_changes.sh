#!/bin/bash
set -e

################# Create a space delimited list of modified modules #################
# Outputs a list of modified modules by comparing changes between main and current commit
# References previous GitHub workflow steps

# Interfacing
if [ "$INTERFACING_CHANGED" == 'true' ]; then
    echo "Detected interfacing changes"
    MODIFIED_MODULES+="interfacing "
fi

# Perception
if [ "$PERCEPTION_CHANGED" == 'true' ]; then
    echo "Detected perception changes"
    MODIFIED_MODULES+="perception "
fi

# Behaviour
if [ "$BEHAVIOUR_CHANGED" == 'true' ]; then
    echo "Detected behaviour changes"
    MODIFIED_MODULES+="behaviour "
fi

# Simulation (Isaac skipped in CI; mjlab compose is simulation_mj)
if [ "$SIMULATION_CHANGED" == 'true' ]; then
    echo "Detected simulation changes"
    MODIFIED_MODULES+="simulation_isaac simulation_mj "
fi

# Embedded
if [ "$EMBEDDED_CHANGED" == 'true' ]; then
    echo "::notice:: Detected embedded changes"
    MODIFIED_MODULES+="embedded "
fi

# Infrastructure sentinel: rebuild all modules (used by docker_context.sh)
if [ "$INFRASTRUCTURE_CHANGED" == 'true' ]; then
    echo "::notice:: Detected infrastructure changes"
    MODIFIED_MODULES="infrastructure"
fi

echo "::notice:: MODIFIED_MODULES are $MODIFIED_MODULES"

# Output list
echo "modified_modules=$MODIFIED_MODULES" >> "$GITHUB_OUTPUT"
