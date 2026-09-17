import yaml

with open("src/interfacing/joint_command/config/hardware_mapping.yaml", "r") as f:
    data = yaml.safe_load(f)

print(data)