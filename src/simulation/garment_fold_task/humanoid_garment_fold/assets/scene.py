# Adapted from the LeHome Challenge (https://github.com/lehome-official/lehome-challenge)
# @ a805ad2f7ab52a4583066fc4ee5180459a7f9d15, Apache License, Version 2.0.
#   source/lehome/lehome/assets/scenes/bedroom.py + utils/constant.py
# MODIFIED (WATonomous): resolves the scene USD from this package's
# `vendor_assets/` instead of a git-root `Assets/` tree.
"""Scene asset paths for the garment-fold task.

`Scene_00_Apartment.usd` (~19 MB) is .gitignore'd — copy it from the LeHome
`Assets/scenes/marble/` or `hf download lehome/asset_challenge`. When loaded
through the full Isaac Sim pipeline it composes the photoreal apartment (NuRec
backdrop + a table); the garment rests on the table.
"""
import os

_SCENES_DIR = os.path.join(
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")),
    "vendor_assets",
    "scenes",
)

MARBLE_BEDROOM_USD_PATH = os.path.join(_SCENES_DIR, "marble", "Scene_00_Apartment.usd")

# Standalone fallback table (committed, ~1.2 MB) — used by GarmentPioneerEnv when
# the apartment USD above is absent.
TABLE038_USD_PATH = os.path.join(_SCENES_DIR, "Table038", "Table038.usd")
