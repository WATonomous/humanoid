## ----------------------- Watod2 Configuration File Override ----------------------------
## ACTIVE PROFILES CONFIGURATION
## List of active profiles to run, defined in docker-compose.yaml.
##
## Possible values:
##   - vis_tools        :   starts tools for data visualization (foxglove)
##   - gazebo           :   starts robot simulator (gazebo)
##   - samples          :   starts up sample nodes for reference

ACTIVE_PROFILES="samples"


## Name to append to docker containers. DEFAULT = <your_watcloud_username>

# COMPOSE_PROJECT_NAME=""


## Tag to use. Images are formatted as <IMAGE_NAME>:<TAG> with forward slashes replaced with dashes.
## DEFAULT = <your_current_github_branch> 

# TAG=""
