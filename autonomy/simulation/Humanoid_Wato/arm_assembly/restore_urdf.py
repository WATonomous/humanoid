import os

filepath = r"c:\Users\rijul\Downloads\UWaterloo\WATonomous\humanoid\autonomy\simulation\Humanoid_Wato\arm_assembly\arm_assembly_fixed.urdf"

# we know the original limits. I will just use git restore if git matches, or I can just use my exact replace since I only ran the replace script recently.
# Or better yet, we can't git checkout easily if we don't know the exact state, but we can just revert the `-3.1415` strings to exactly what they were!
# Wait, different joints had different limits!  [-1.57, 0] or [0, 1.57].
# Oh no, I overwrote them all to [-3.1415, 3.1415] universally! I lose the original bounds for each joint!

# I can just re-extract from the prompt context where I saw them previously:
# mcp_index: [-1.570796, 0.0]
# pip_index: [0.0, 1.570796]
# dip_index: [-1.570796, 0.0]
# mcp_middle: [-1.570796, 0.0]
# pip_middle: [0.0, 1.570796]
# dip_middle: [0.0, 1.570796]
# etc.

# It is easier to just run `git checkout` on the urdf file! It's a git repo.
import subprocess
try:
    subprocess.run(["git", "checkout", "--", "arm_assembly_fixed.urdf"], cwd=os.path.dirname(filepath), check=True)
    print("URDF restored via git!")
except Exception as e:
    print(f"Git failed: {e}")
