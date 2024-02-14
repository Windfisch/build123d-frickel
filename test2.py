# %%

from build123d import *
from ocp_vscode import *

import ocp_vscode

Loc = Location
Rot = Rotation
Rect = Rectangle
RRect = RectangleRounded

for x,xx in [('L', Align.MIN), ('C', Align.CENTER), ('H', Align.MAX)]:
    for y,yy in [('L', Align.MIN), ('C', Align.CENTER), ('H', Align.MAX)]:
        globals()[x+y] = (xx, yy)
        for z,zz in [('L', Align.MIN), ('C', Align.CENTER), ('H', Align.MAX)]:
            globals()[x+y+z] = (xx, yy, zz)

set_defaults(black_edges=True, render_joints=True, render_edges=True, reset_camera=False, default_opacity=1)


# %%

from enum import Enum, auto

# auto_finger_joint code taken from ZTKF

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


def servo_horn_mount():
    a = Loc((-6, 0)) * Circle(7)
    b = Loc((20,0)) * Rectangle(24,20, align=LC)
    c = make_hull([a.edges(),b.edges()])
    holes = Location((-9,0)) * SlotCenterToCenter(2.3, 1.7)
    holes += Location((14,0)) * SlotCenterToCenter(2.3, 1.7)
    holes += RRect(10, 8, 1)

    r = c - holes

    r = extrude(r, 3)

    joint_point = (
        r.faces().filter_by(Plane.XZ).sort_by(Axis.Y)[0]
        .edges().filter_by(Axis.X).sort_by(Axis.Y)[-1]
        .vertices().sort_by(Axis.X)[-1]
    )

    print(joint_point)

    j1 = RevoluteJoint("knee_servo_horn", r, Axis((0,0,0), (0,0,1)))
    j2 = RigidJoint("hip_servo_mount", r, -Loc(joint_point, (0,0,180)))

    return r

def servo_hip_mount():
    d = 12
    round = 4
    servo_xlen = 22.7
    servo_ylen = 12.0

    yextra = 12

    
    base = Loc((-d,0)) * (RRect(d + servo_xlen + 5, servo_ylen + yextra, round, align = LC) + Rect(round + 0.1, servo_ylen + yextra, align = LC))
    base -= RRect(servo_xlen, servo_ylen, 0.5, align=LC)

    base = extrude(base, 3)

    joint_loc = base.vertices().group_by(Axis.X)[0].group_by(Axis.Y)[0].sort_by(Axis.Z)[0]
    print(joint_loc)

    tri1_loc = Vector(joint_loc + (0,3/2,0))
    tri2_loc = tri1_loc.__copy__()
    tri2_loc.Y *= -1

    j1 = RigidJoint("attach", base, Loc(joint_loc, (0,90,90)))
    j2 = RigidJoint("tri1", base, Loc(tri1_loc, (90,0,0)))
    j2 = RigidJoint("tri2", base, Loc(tri2_loc, (90,0,0)))

    return base

def tri():
    xlen = 30
    ylen = 17
    tol = 1
    
    tri = Triangle(a=xlen-tol, c=ylen-tol, B = 90, align = LL) + Rect(xlen-tol, 3+tol, align=LH) + Rect(3+tol, ylen-tol, align = HL)
    tri = Loc((tol, tol)) * tri
    tri = extrude(tri, 3/2, both=True)
    j = RigidJoint("attach", tri, Loc((-3,-3,0), (0,0,0)))

    return tri

x,y,t1,t2 = None,None,None,None
    

x = servo_horn_mount()
y = Rot((90,0,90)) * servo_hip_mount()
t1 = tri()
t2 = tri()

x.joints['hip_servo_mount'].connect_to(y.joints['attach'])

y.joints['tri1'].connect_to(t1.joints['attach'])
y.joints['tri2'].connect_to(t2.joints['attach'])

xtrans = x.location.inverse()
ytrans = y.location.inverse()
t1trans = t1.location.inverse()
t2trans = t2.location.inverse()


finger = auto_finger_joint

x, y = finger(x, y, 3)
x, t1 = finger(x, t1, 3, swap=True)
x, t2 = finger(x, t2, 3, swap=True)
y, t1 = finger(y, t1, 3, swap=True)
y, t2 = finger(y, t2, 3, swap=True)

x.color='red'
y.color='blue'

x.name = 'leg_servo_horn'
y.name = 'hip_servo_mount'
t1.name = 'tri1'
t2.name = 'tri2'


x.location = Loc((0,30,0)) * xtrans
y.location = ytrans
t1.location = Loc((50,-10,0)) * t1trans
t2.location = Loc((50,20,0)) * t2trans


part = x+y+t1+t2

part2d = section(part, Plane.XY)

show(part2d)

for f in part2d.faces():
    

exporter = ExportSVG(scale=1)
exporter.add_layer("Visible")
exporter.add_shape(part2d, layer="Visible")
exporter.write("part_projection.svg")
