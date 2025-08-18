import cadquery as cq
# from cadquery import exporters
from cadquery.occ_impl.exporters import *
import numpy as np
import re
import os

start = [0., 0., 0.]
end = [0., 1., 0.]

line_width = 0.4
layer_height = 0.2
eps = 1e-8

path_gcode = r'path-to-gcode'

accuracy = 'fillet' # fillet, chamfer, simple

def rectangle_points(s, e, width):
    """
    Returns four edge points (A, B, C, D) of a rectangle
    given the coordinates of the centerline (start, end)
    and the width of the rectangle
    """
    u = e - s
    u = u/np.linalg.norm(u)
    v = np.array([-u[1], u[0]])
    w = 0.5*width
    rx = w*v[0]
    ry = w*v[1]
    A = (s[0]+rx, s[1]+ry)
    B = (s[0]-rx, s[1]-ry)
    C = (e[0]-rx, e[1]-ry)
    D = (e[0]+rx, e[1]+ry)
    return (A, B, C, D)

def extrusion_line(start, end, line_width, layer_height, eps=1e-8, accuracy='fillet'):
    """
    Returns a workplane object with 
    """
    # Calculate new start end end points of the rectangle
    s = np.array(start)
    e = np.array(end)
    length_line = np.linalg.norm(e-s)
    # Circle
    if length_line<=eps:
        circle = cq.Workplane('XY').circle(line_width/2.).extrude(layer_height).translate((s[0], s[1], s[2]))
        return circle
    ext = line_width/2.
    f = ext/length_line
    new_startpoint = s-f*(e-s)
    new_endpoint = e+f*(e-s)
    # Rectangle
    rct_pnts = rectangle_points(new_startpoint, new_endpoint, line_width)
    if accuracy=='simple':
        rectangle = cq.Workplane('XY').polyline(rct_pnts).close().extrude(layer_height).translate((0., 0., new_endpoint[2]))
    elif accuracy=='fillet':
        rectangle = cq.Workplane('XY').polyline(rct_pnts).close().extrude(layer_height).edges("|Z").fillet(line_width/2.-eps).translate((0., 0., new_endpoint[2]))
    elif accuracy=='chamfer':
        x = np.sqrt(2.)*((np.sqrt(2)-1)*ext)
        rectangle = cq.Workplane('XY').polyline(rct_pnts).close().extrude(layer_height).edges("|Z").chamfer(x).translate((0., 0., new_endpoint[2]))
    return rectangle

def group_into_layers(gcode_path):
    with open(gcode_path, 'r') as file:
        lines = file.readlines()

    chunks = []
    current_chunk = []
    found_first_change_layer = False

    for line in lines:
        if line.startswith('; CHANGE_LAYER'):
            found_first_change_layer = True
            if current_chunk:
                chunks.append(current_chunk)
                current_chunk = []
        if found_first_change_layer:
            current_chunk.append(line)

    if current_chunk:
        chunks.append(current_chunk)

    return chunks

def extract_coordinates(line):
    if not line.startswith('G1'):
        return ('skip',)
    pattern_f = r'G1 F(?P<e>[0-9.]+)'
    match = re.search(pattern_f, line)
    if match:
        return ('skip',)
    pattern_extrusion = r'G1 X(?P<x>[0-9.]+) Y(?P<y>[0-9.]+) E(?P<e>[0-9.]+)'
    match = re.search(pattern_extrusion, line)
    if match:
        x = float(match.group('x'))
        y = float(match.group('y'))
        return ('extrusion', (x, y))
    pattern_extrusion_arc = r'G2 X(?P<x>[0-9.]+) Y(?P<y>[0-9.]+) I-(?P<i>[0-9.]+) J(?P<j>[0-9.]+) E(?P<e>[0-9.]+)'
    match = re.search(pattern_extrusion_arc, line)
    if match:
        x = float(match.group('x'))
        y = float(match.group('y'))
        i = float(match.group('i'))
        j = float(match.group('j'))
        return ('arc', (x, y, i, j))
    pattern_travel = r'G1 X(?P<x>[0-9.]+) Y(?P<y>[0-9.]+) F(?P<e>[0-9.]+)'
    match = re.search(pattern_travel, line)
    if match:
        x = float(match.group('x'))
        y = float(match.group('y'))
        return ('travel', (x, y))
    pattern_retraction = r'G1 X(?P<x>[0-9.]+) Y(?P<y>[0-9.]+) E-(?P<e>[0-9.]+)'
    match = re.search(pattern_retraction, line)
    if match:
        x = float(match.group('x'))
        y = float(match.group('y'))
        return ('retraction', (x, y))
    pattern_only_retraction = r'G1 E(?P<e>[0-9.]+)'
    match = re.search(pattern_only_retraction, line)
    if match:
        return ('circle',)
    pattern_only_retraction_neg = r'G1 E(?P<e>-?[0-9.]+)'
    match = re.search(pattern_only_retraction_neg, line)
    if match:
        return ('skip',)
    pattern_z_move = r'G1 X(?P<x>[0-9.]+) Y(?P<y>[0-9.]+) Z(?P<e>[0-9.]+)'
    match = re.search(pattern_z_move, line)
    if match:
        x = float(match.group('x'))
        y = float(match.group('y'))
        return ('travel', (x, y))
    pattern_only_z_move = r'G1 Z(?P<e>-?[0-9.]+)'
    match = re.search(pattern_only_z_move, line)
    if match:
        return ('skip',)
    return False

layers = group_into_layers(path_gcode)


# line = extrusion_line(start, end, line_width, layer_height)
# show_object(line)

extrusions = []
current_height = 0.
start = np.array([0., 0., 0.])
flow_rate = 1.0
for layer in layers:
    extrusions_layer = []
    layer_height = float(layer[2].split(' ')[-1])
    for line in layer:
        # Skip comments
        if line.startswith(';'):
            # Flow rate modifier
            if line.startswith('; FEATURE: Internal Bridge'):
                flow_rate = 1.15
            elif line.startswith('; FEATURE:'):
                flow_rate = 1.0
            # Extract line width
            if line.startswith('; LINE_WIDTH:'):
                pattern = r'; LINE_WIDTH: (?P<lw>[0-9.]+)'
                match = re.search(pattern, line)
                lw = float(match.group('lw'))
                if match:
                    extrusions_layer.append(['lw', lw*flow_rate])
            continue
        # Skip machine settings
        elif line.startswith('M'):
            continue
        # Skip comments that start with space
        elif line.startswith(' '):
            continue
        # Extract coordinates
        coords = extract_coordinates(line)
        if not coords:
            continue
            print('Error', line)
        else:
            if coords[0]=='skip':
                continue
        if coords[0]=='travel':
            x, y = coords[1]
            start = np.array([x, y, current_height])
        elif coords[0]=='extrusion':
            x, y = coords[1]
            end = np.array([x, y, current_height])
            extrusions_layer.append([start, end])
            start = end
        elif coords[0]=='circle':
            end = start
            extrusions_layer.append([start, end])
            start = end
    extrusions.append(extrusions_layer)
    current_height += layer_height

# Fast hack
extrusions[1] = extrusions[1][1:]

# result = cq.Workplane('XY')

filename = os.path.splitext(os.path.basename(path_gcode))[0]
folder_path = os.path.dirname(path_gcode)
# name = '{}.step'.format(filename)
# filepath_step = os.path.join(folder_path, name)

filepaths = []
for index, extrusion_layer in enumerate(extrusions[:1]):
    result_layer = cq.Workplane('XY')
    print('Layer {}/{} ...'.format(index+1, len(extrusions)))
    for extrusion in extrusion_layer:
        try:
            if extrusion[0]=='lw':
                line_width = extrusion[1]
                continue
            line = extrusion_line(extrusion[0], extrusion[1], line_width, layer_height, accuracy=accuracy)
            # result = result.union(line)
            result_layer = result_layer.union(line)
        except:
            pass
    name = '{}-{}.step'.format(filename, index+1)
    filepath_step = os.path.join(folder_path, name)
    with open(filepath_step, "w") as fp:
        cq.exporters.exportShape(result_layer, ExportTypes.STEP, fp)
    filepaths.append(filepath_step)


# Combine into one
# result = cq.Workplane('XY')

# for i, filepath in enumerate(filepaths):
    # layer = cq.occ_impl.importers.importStep(filepath)
    # layer = layer.translate((0., 0., -i*eps))
    # result = result.union(layer)
    # os.remove(filepath)

# name = '{}.step'.format(filename)
# filepath_step = os.path.join(folder_path, name)

# with open(filepath_step, "w") as fp:
#     cq.exporters.exportShape(result, ExportTypes.STEP, fp)




