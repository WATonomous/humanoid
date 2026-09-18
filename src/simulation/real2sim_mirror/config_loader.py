import yaml
import math

#creating a lookup table for easy access
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

#angle computation function so rad angles can be sent to mjlabs
def angle_computation(motor_id, raw_position, lookup):
    if motor_id not in lookup:
        print (f"Warning: Unknown Motor ID {motor_id}")
        return None
        
    config = lookup[motor_id]
    zero_offset = config["zero_offset"]
    direction =  config["direction"]

    true_angle_deg = (raw_position - zero_offset) * direction
    true_angle_rad = math.radians(true_angle_deg)

    return true_angle_rad

if __name__ == "__main__":
    yaml_file_path = "src/joint_command/config/hardware_mapping.yaml" 
    lookup_table = load_hardware_mapping(yaml_file_path)
    print(lookup_table)

    test_angle = angle_computation(14, 1900, lookup_table)
    print(test_angle)
