# %%

from build123d import *
from ocp_vscode import *

import ocp_vscode

Loc = Location
Rot = Rotation
Rect = Rectangle
RRect = RectangleRounded

def rot2d(angle):
    return Rotation(0,0,angle)

for x,xx in [('L', Align.MIN), ('C', Align.CENTER), ('H', Align.MAX)]:
    for y,yy in [('L', Align.MIN), ('C', Align.CENTER), ('H', Align.MAX)]:
        globals()[x+y] = (xx, yy)
        for z,zz in [('L', Align.MIN), ('C', Align.CENTER), ('H', Align.MAX)]:
            globals()[x+y+z] = (xx, yy, zz)

set_defaults(black_edges=True, render_joints=True, render_edges=True, reset_camera=False, default_opacity=0.7)

# auto_finger_joint code taken from ZTKF
from enum import Enum, auto
class FingerType(Enum):
    ODD = auto()
    EVEN = auto()

from build123d import sqrt
from build123d import floor
from itertools import product
from build123d import sin
from build123d import pi
from numpy import linspace

def auto_finger_joint(
        a: Part,
        b: Part,
        min_finger_width: float,
        swap: bool = False,
        finger_type: FingerType = None
    ) -> tuple[Part, Part]:

    # We're operating on the intersection of the two parts
    inter = a.intersect(b)
    if len(inter.faces()) == 0:
        return a,b
    
    edges = inter.edges().copy()
    edges.sort(key=lambda e: e.length, reverse=True)

    # The operation will be along the shortest of the longest 4
    # edges in the direction of the edge
    edge = edges[0]
    z_dir = (edge @ 1 - edge @ 0).normalized()

    # Determine the number of fingers, one is added to the base
    # count since there is technically a 0th cut. That flips some
    # of the even/odd logic 
    n_fingers = floor(edge.length/min_finger_width) + 1
    if finger_type == FingerType.EVEN and not n_fingers & 1:
        n_fingers -= 1
    elif finger_type == FingerType.ODD and n_fingers & 1:
        n_fingers -= 1
    
    # These are the arrays we'll be filling
    fingers_a, fingers_b = [], []

    # We'll use linspace to evenly space the fingers, skip the
    # first and last because they're outside the intersection
    alternate = (fingers_a, fingers_b)
    to_div = inter

    # 1 is added here since 
    for x in linspace(0.0, 1.0, n_fingers)[1:-1]:

        # Split by our plane along the edge
        plane = Plane(origin=edge @ x, z_dir=z_dir)
        divs = [shape for shape in to_div.split(plane, Keep.BOTH)]

        # Select the correct bottom/top
        if plane.to_local_coords(divs[0]).center().Z >= 0:
            alternate[0].append(divs[1])
            to_div = divs[0]
        else:
            alternate[0].append(divs[0])
            to_div = divs[1]

        # Swap the arrays
        alternate = (alternate[1], alternate[0])

    # The remainder will be the last finger
    alternate[0].append(to_div)

    if swap:
        return (a - fingers_b, b - fingers_a)
    else:
        return (a - fingers_a, b - fingers_b)

# %%

foo = Box(30, 30, 10)
foo -= Box(10,10,10)
foo += Loc((10,10,10))*Box(10,10,10)
foo -= Loc((-12.5,-12.5,0)) * Box(5,5,10)

edge_to_chamfer = foo.edges().group_by(Axis.Y)[-1].sort_by(Axis.X)[0]
#foo = chamfer(edge_to_chamfer, 10)

cuts = [face.thicken(-0.5) for face in foo.faces()]

for i in range(0, len(cuts)):
    for j in range(i+1, len(cuts)):
        print(i,j)
        cuts[i], cuts[j] = auto_finger_joint(cuts[i], cuts[j], 2)


# %%        


def laserize2(cut):
    faces = cut.faces().sort_by(SortBy.AREA)[-2:]

    solid0 = extrude(faces[0], -0.5)
    solid1 = extrude(faces[1], -0.5)

    return solid0.intersect(solid1)

def laserize3(cut):
    face = cut.faces().sort_by(SortBy.AREA)[-1]
    #plane = Plane(face.center_location)
    plane = Plane.YZ

    projected = project(cut.face(), plane, mode=Mode.ADD)
    #print(projected)

    return cut

def laserize(cut):
    face = cut.faces().sort_by(SortBy.AREA)[-1]
    plane = Plane(Rot((1,1,1)) * face.center_location)
    #plane = Plane.YZ
    for f in cut.faces():
        print(Axis((0,0,0), f.normal_at()).is_normal(plane.location.z_axis))
        print(Axis((0,0,0), f.normal_at()).angle_between(plane.location.z_axis))
        
    #moveloc = Location(-plane.location.z_axis.direction)

    projected = project([f for f in cut.faces() if not Axis((0,0,0), f.normal_at()).is_normal(plane.location.z_axis)], plane)
    print(projected)

    return cut


#cuts2 = [laserize(cut) for cut in cuts]
cuts2=cuts
import random

for cut in cuts2:
    cut.color = Color(random.random(), random.random(), random.random())

show(cuts2)


# %%
