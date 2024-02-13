# %%

from build123d import *
from ocp_vscode import *

# %%


a = Cylinder(2,3)
a.color = '#ffaaaa'
a.label = "a"
b = Cylinder(1.5,3.1)
b.color = '#aaffaa'
b.label="b"
c = Cylinder(1,3.2)
c.color = '#aaaaff'
c.label="c"


a1 = RigidJoint("a1", a, Plane(a.faces().sort_by(Axis.Z)[-1]).location)
a2 = RigidJoint("a2", a, -Plane(a.faces().sort_by(Axis.Z)[0]).location)
b1 = RigidJoint("b1", b, Plane(b.faces().sort_by(Axis.Z)[-1]).location)
b2 = RigidJoint("b2", b, -Plane(b.faces().sort_by(Axis.Z)[0]).location)
c1 = RigidJoint("c1", c, Plane(c.faces().sort_by(Axis.Z)[-1]).location)
c2 = RigidJoint("c2", c, -Plane(c.faces().sort_by(Axis.Z)[0]).location)

c2.connect_to(b1)
b2.connect_to(a1)

show(a,b,c)
# %%
