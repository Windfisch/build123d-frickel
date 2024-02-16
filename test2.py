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


# %%

from enum import Enum, auto

THICK = 3.1

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
    a = Loc((-3, 0)) * Circle(10)
    b = Loc((14,0)) * Rectangle(24,20, align=LC)
    c = make_hull([a.edges(),b.edges()])
    holes = Location((-8.5,0)) * SlotCenterToCenter(1.3, 2.1)
    holes += Location((8.5,0)) * SlotCenterToCenter(1.3, 2.1)
    holes += Location((0,7)) * rot2d(90) * SlotCenterToCenter(1.3, 2.1)
    holes += Location((0,-7)) * rot2d(90) * SlotCenterToCenter(1.3, 2.1)
    holes += RRect(10, 7.7, 1)

    r = c - holes

    r = extrude(r, THICK)

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
    servo_xlen = 23
    servo_ylen = 12.0
    screw_spacing = 28

    yextra = 12

    
    base = Loc((-d,0)) * (RRect(d + servo_xlen + 5, servo_ylen + yextra, round, align = LC) + Rect(round + 0.1, servo_ylen + yextra, align = LC))
    base -= RRect(servo_xlen, servo_ylen, 0.5, align=LC)
    base -= Loc((servo_xlen/2 -screw_spacing/2,0)) * Circle(1.5/2)
    base -= Loc((servo_xlen/2 +screw_spacing/2,0)) * Circle(1.5/2)

    base = extrude(base, THICK)

    joint_loc = base.vertices().group_by(Axis.X)[0].group_by(Axis.Y)[0].sort_by(Axis.Z)[0]
    print(joint_loc)

    tri1_loc = Vector(joint_loc + (0,THICK/2,0))
    tri2_loc = tri1_loc.__copy__()
    tri2_loc.Y *= -1
    servo_loc = Vector(servo_xlen/2, 0, THICK)

    RigidJoint("attach", base, Loc(joint_loc, (0,90,90)))
    RigidJoint("tri1", base, Loc(tri1_loc, (90,0,0)))
    RigidJoint("tri2", base, Loc(tri2_loc, (90,0,0)))
    RigidJoint("servo", base, Loc(servo_loc, (180,0,180)))

    return base

def tri():
    xlen = 30
    ylen = 17
    tol = 2
    
    tri = (
        Triangle(a=xlen-tol, c=ylen-tol, B = 90, align = LL)
        + Rect(xlen-tol, THICK+tol, align=LH)
        + Rect(THICK+tol, ylen-tol, align = HL)
        + Rect(tol, tol, align = HH)
        - Loc((xlen-2*tol,-tol)) * Rect(tol, THICK, align = LH)
        - Loc((-tol, ylen-2*tol)) * Rect(THICK, tol, align = HL)
    )
    tri = Loc((tol, tol)) * tri
    tri = extrude(tri, THICK/2, both=True)
    j = RigidJoint("attach", tri, Loc((-THICK,-THICK,0), (0,0,0)))

    return tri


def servo():
    body_xlen = 23
    body_ylen = 12.5
    body_zlen = 24
    
    hole_xdist = 27.7

    flange_xlen = 34
    flange_zlen = 2.5
    flange_zdist_bottom = 17.3

    axis_offcenter = 5.5

    result = extrude(Rect(body_xlen, body_ylen), body_zlen)
    flange = extrude(
        Rect(flange_xlen, body_ylen)
        - Loc((-hole_xdist/2,0)) * (Circle(3/2) + Rect(99, 1.2, align=HC))
        - Loc((+hole_xdist/2,0)) * (Circle(3/2) + Rect(99, 1.2, align=LC)),
        flange_zlen
    )
    flange = Loc((0,0,flange_zdist_bottom)) * flange
    result += flange

    topaxis = Plane(Loc((-axis_offcenter,0,0)) * result.faces().sort_by(Axis.Z)[-1].center_location)

    result += extrude( topaxis * Circle(11.15/2), 4)
    result += extrude( topaxis * Circle(5/2), 6.5)
    result += extrude( topaxis * Loc((11.15/2 + 3.35,0)) * Circle(2.75, align=HC), 4)

    RigidJoint("mount", result, flange.faces().sort_by(Axis.Z)[-1].center_location)
    RevoluteJoint("horn_master", result, Axis(result.faces().sort_by(Axis.Z)[-1].center_location.position, (0,0,1)))
    RigidJoint("horn_slave", result, result.faces().sort_by(Axis.Z)[-1].center_location)

    result.name="sg90 servo"
    result.color="#5599ffb0"

    return result

def servo_horn():
    xlen = 36

    end_r = 2
    mid_r = 7.5 / 2

    base = Circle(mid_r) + Loc((-xlen/2 +end_r,0)) * Circle(end_r) + Loc((xlen/2-end_r,0)) * Circle(end_r)
    base = make_hull(base.edges())
    
    hole_spacing = 2.5
    outer_hole_dist = 32
    hole_diam = 1.3
    n_holes = 5

    for i in range(n_holes):
        base -= Loc((outer_hole_dist/2 - i * hole_spacing,0)) * Circle(hole_diam/2)
        base -= Loc((-outer_hole_dist/2 + i * hole_spacing,0)) * Circle(hole_diam/2)
    
    base -= Circle(1.2)
    
    result = extrude(base, 2)
    pos = (0,0,1)
    RigidJoint('slave', result, Loc(pos, (0,0,1)))
    RevoluteJoint('master', result, Axis(pos, (0,0,1)))
    RigidJoint('mount', result, Loc((0,0,2), (0,0,0)))

    result.name = "servo horn"
    result.color = "#ffffffd0"

    return result




x,y,t1,t2 = None,None,None,None

x = servo_horn_mount()
y = Rot((90,0,90)) * servo_hip_mount()
t1 = tri()
t2 = tri()

servo1 = servo()
servo2 = servo()
horn1 = servo_horn()

x.joints['hip_servo_mount'].connect_to(y.joints['attach'])
x.joints['knee_servo_horn'].connect_to(horn1.joints['mount'], angle=0)

horn1.joints['master'].connect_to(servo2.joints['horn_slave'], angle=180+30)

y.joints['tri1'].connect_to(t1.joints['attach'])
y.joints['tri2'].connect_to(t2.joints['attach'])

y.joints['servo'].connect_to(servo1.joints['mount'])

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

show(x,y,t1,t2, servo1, servo2, horn1)


#x.location = Loc((0,30,0)) * xtrans
#y.location = ytrans
#t1.location = Loc((50,-10,0)) * t1trans
#t2.location = Loc((50,20,0)) * t2trans


#part = x+y+t1+t2

#part2d = section(part, Plane.XY)

#show(part2d)

#exporter = ExportSVG(scale=1)
#exporter.add_layer("Visible")
#exporter.add_shape(part2d, layer="Visible")
#exporter.write("part_projection.svg")
