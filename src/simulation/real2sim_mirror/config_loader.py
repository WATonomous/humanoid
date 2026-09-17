import yaml

def load_hardware_mapping(yaml_file_path):
    with open(yaml_file_path, "r") as f:
        data = yaml.safe_load(f)

    lookup_table = {}

    for side, limbs in data.items():
        for limb, joints in limbs.items():
            for joint_type, config in joints.items():
                joint_name = f"{side}_{limb}_{joint_type}"
                entry = config.copy()
                entry["joint_name"] = joint_name
                lookup_table[config["can_id"]] = entry

    return lookup_table

if __name__ == "__main__":
    yaml_file_path = "src/interfacing/joint_command/config/hardware_mapping.yaml" 
    lookup_table = load_hardware_mapping(yaml_file_path)
    print(lookup_table)
