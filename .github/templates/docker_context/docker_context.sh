#!/bin/bash
set -e

################# Sweep for Docker Services and Modules #################
# Scans for services and modules in the wato_monorepo,
# dynamically builds a json matrix for downstream CI build and testing

# Find docker compose files in 'modules' directory
modules=$(find ${GITHUB_WORKSPACE}/modules -maxdepth 1 -name "docker-compose*")

# Initialize an empty array for JSON objects
json_objects=()

# Check for infrastructure changes
TEST_ALL=false
MODIFIED_MODULES="infrastructure"
if [[ $MODIFIED_MODULES = "infrastructure" ]]; then
    TEST_ALL=true
    echo Testing all "$MODULES_DIR"
fi

# Loop through each module
while read -r module; do
    # Retrieve docker compose service names
    services=$(docker compose -f "$module" --profile deploy --profile develop config --services)
    module_out=$(basename $(echo "$module" | sed -n 's/modules\/docker-compose\.\(.*\)\.yaml/\1/p'))
    # Skip simulation module
    if [[ 'simulation' = $module_out ]]; then
        continue
    fi
    # Only work with modules that are modified
    if [[ $MODIFIED_MODULES != *$module_out* && $TEST_ALL = "false"  ]]; then
        continue
    fi
    # Loop through each service
    while read -r service_out; do
        # Temporarily skip perception services that have too large image size
        # if  [[ "$service_out" == "lane_detection" ]] || \
        #     [[ "$service_out" == "camera_object_detection" ]] || \
        #     continue
        # fi
        # Construct JSON object for each service with module and service name
        # TODO: Expose whole profile object to env
        dockerfile=$(docker compose -f "$module" --profile deploy --profile develop config | yq ".services.$service_out.build.dockerfile")
        json_object=$(jq -nc --arg module_out "$module_out" --arg service_out "$service_out" --arg dockerfile "$dockerfile" \
        '{module: $module_out, service: $service_out, dockerfile: $dockerfile}')
        # Append JSON object to the array
        json_objects+=($json_object)
        
    done <<< "$services"
done <<< "$modules"
# Convert the array of JSON objects to a single JSON array
json_services=$(jq -nc '[( $ARGS.positional[] | fromjson )]' --args -- ${json_objects[*]})

if [ -z $json_services ]; then
    exit 1
fi

echo output "docker_matrix=$(echo $json_services | jq -c '{include: .}')"
echo "docker_matrix=$(echo $json_services | jq -c '{include: .}')" >> $GITHUB_OUTPUT

################# Setup Docker Registry and Repository Name #################
# Docker Registry to pull/push images
REGISTRY_URL="ghcr.io/watonomous/humanoid"

REGISTRY=$(echo "$REGISTRY_URL" | sed 's|^\(.*\)/.*$|\1|')
REPOSITORY=$(echo "$REGISTRY_URL" | sed 's|^.*/\(.*\)$|\1|')

echo "registry=$REGISTRY" >> $GITHUB_OUTPUT
echo "repository=$REPOSITORY" >> $GITHUB_OUTPUT