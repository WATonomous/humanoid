from isaaclab_tasks.utils import import_packages

# The blacklist is used to prevent importing configs from sub-packages
_BLACKLIST_PKGS = [
    "utils",
    "dextrah",   # requires RslRlPpoActorCriticRecurrentCfg which is not in this isaaclab_rl build
]
import_packages(__name__, _BLACKLIST_PKGS)
