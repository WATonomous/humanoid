## ----------------------- watod Configuration File Override ----------------------------

##
## HINT: You can copy the contents of this file to a watod-config.local.sh 
##       file that is untrackable by git and readable by watod.
##

## ----------------------- watod Configuration File Override ----------------------------
## ACTIVE Modules CONFIGURATION
## List of active modules to run, defined in docker-compose.yaml.
##
## Possible values:
##   - infrastructure     	:   starts visualization tools
##   - interfacing          :   starts interfacing nodes
##	 - perception			:	starts perception nodes
##	 - controller		    :	starts controller nodes
##	 - simulation			:	starts simulation
##   - samples             	:   starts sample ROS2 pubsub nodes

ACTIVE_MODULES=""

################################# MODE OF OPERATION #################################
## Possible modes of operation when running watod.
## Possible values:
##	 - deploy (default)		:	runs production-grade containers (non-editable)
##	 - develop   		    :	runs developer containers (editable)

# MODE_OF_OPERATION=""

############################## ADVANCED CONFIGURATIONS ##############################
## Name to append to docker containers. DEFAULT = "<your_watcloud_username>"
# COMPOSE_PROJECT_NAME=""

## Tag to use. Images are formatted as <IMAGE_NAME>:<TAG> with forward slashes replaced with dashes.
## DEFAULT = "<your_current_github_branch>"
# TAG=""

# Docker Registry to pull/push images. DEFAULT = "ghcr.io/watonomous/wato_monorepo"
# REGISTRY_URL=""

## Platform in which to build the docker images with. 
## Either arm64 (apple silicon, raspberry pi) or amd64 (most computers)
# PLATFORM="amd64"
