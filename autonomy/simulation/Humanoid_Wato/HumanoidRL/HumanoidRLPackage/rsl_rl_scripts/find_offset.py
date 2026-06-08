import sys
import argparse
from isaaclab.app import AppLauncher

# We have to launch Isaac Lab headlessly first, otherwise the Pixar library
# doesn't know how to download files from NVIDIA's cloud servers!
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf

# The exact path used by Isaac Lab for the cabinet
usd_path = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2.0/Isaac/Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd"

print(f"Downloading and opening USD: {usd_path}")
print("(This may take a few seconds...)\n")

stage = Usd.Stage.Open(usd_path)

if not stage:
    print("Could not open USD file!")
    sys.exit(1)

print("===== FOUND PRIMS RELATED TO THE TOP DRAWER =====")
for prim in stage.Traverse():
    name = prim.GetName().lower()
    if "drawer" in name or "handle" in name:
        # We only care about the top drawer stuff
        if "bottom" not in name:
            print(f"Path: {prim.GetPath()}")
            
            # Extract its mathematical location in the 3D file
            xform = UsdGeom.Xformable(prim)
            if xform:
                time = Usd.TimeCode.Default()
                transform = xform.ComputeLocalToWorldTransform(time)
                translation = transform.ExtractTranslation()
                print(f"   World Position (X, Y, Z): ({translation[0]:.4f}, {translation[1]:.4f}, {translation[2]:.4f})")
            print("-" * 50)

print("\nDONE!")
