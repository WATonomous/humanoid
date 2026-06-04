"""
dump_axes.py
============
A utility script that parses the arm_assembly_fixed.urdf, extracts all finger joint axis
and origin definitions, and prints/saves them. This is useful for debugging finger joint configurations
and ensuring they are aligned correctly.

Process:
  1. Parse the local arm_assembly_fixed.urdf file
  2. Iterate through all joints and find finger joints (containing mcp, pip, or dip in name)
  3. Extract their axis (rotation direction vector) and origin (displacement from parent joint)
  4. Write the results to axes_out.txt in the temporary directory and print them to the terminal
"""
import os
import tempfile
import xml.etree.ElementTree as ET

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_urdf_path = os.path.join(_SCRIPT_DIR, 'arm_assembly', 'arm_assembly_fixed.urdf')
tree = ET.parse(_urdf_path)
lines = []
for j in tree.findall('joint'):
    name = j.attrib.get('name', '')
    if any(x in name for x in ['mcp', 'pip', 'dip']):
        axis = j.find('axis')
        origin = j.find('origin')
        ax = axis.attrib['xyz'] if axis is not None else '?'
        ox = origin.attrib['xyz'] if origin is not None else '?'
        lines.append(f'{name}: axis={ax}  origin_xyz={ox}')

_out_file = os.path.join(tempfile.gettempdir(), 'axes_out.txt')
with open(_out_file, 'w') as f:
    f.write('\n'.join(lines))

for l in lines:
    print(l)
