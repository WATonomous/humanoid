import xml.etree.ElementTree as ET
tree = ET.parse('arm_assembly/arm_assembly_fixed.urdf')
lines = []
for j in tree.findall('joint'):
    name = j.attrib.get('name', '')
    if any(x in name for x in ['mcp', 'pip', 'dip']):
        axis = j.find('axis')
        origin = j.find('origin')
        ax = axis.attrib['xyz'] if axis is not None else '?'
        ox = origin.attrib['xyz'] if origin is not None else '?'
        lines.append(f'{name}: axis={ax}  origin_xyz={ox}')

with open('/tmp/axes_out.txt', 'w') as f:
    f.write('\n'.join(lines))

for l in lines:
    print(l)
