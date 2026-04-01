import re
with open('c:/Users/rijul/Downloads/UWaterloo/WATonomous/humanoid/autonomy/simulation/Humanoid_Wato/arm_assembly/arm_assembly_fixed.urdf', 'r') as f:
    text = f.read()
text = re.sub(r'lower=\".*?\" upper=\".*?\"', 'lower=\"-3.1415\" upper=\"3.1415\"', text)
with open('c:/Users/rijul/Downloads/UWaterloo/WATonomous/humanoid/autonomy/simulation/Humanoid_Wato/arm_assembly/arm_assembly_fixed.urdf', 'w') as f:
    f.write(text)
print("Done fixing URDF limits for DexRetargeting!")
