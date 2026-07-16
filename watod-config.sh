## ----------------------- watod Configuration File Override ----------------------------

##
## HINT: You can copy the contents of this file to a watod-config.local.sh 
##       file that is untrackable by git and readable by watod.
##

## ----------------------- watod Configuration File Override ----------------------------
## ACTIVE Modules CONFIGURATION
## List of active modules to run (each needs modules/docker-compose.<name>.yaml).
##
## Possible values:
##   - interfacing          :   CAN / hardware interfacing
##   - perception           :   perception nodes
##   - behaviour            :   joint_command, voxel_grid
##   - samples              :   sample ROS 2 pub/sub nodes
##   - simulation_isaac     :   Isaac Lab (SO101 IL, HumanoidRL, Quest teleop)
##   - simulation_mj        :   MuJoCo / mjlab (mjlabs service)
##   - simulation_mj.cpu    :   MuJoCo / mjlab CPU-only (mjlabs service)

ACTIVE_MODULES="samples"

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
