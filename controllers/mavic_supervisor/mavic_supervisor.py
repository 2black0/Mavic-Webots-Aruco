"""mavic_supervisor controller."""

from controller import Supervisor
import sys

# import math3d as m3d

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

drone = supervisor.getFromDef("drone")
if drone is None:
    sys.stderr.write("No DEF supervisor node found in the current world file\n")
    sys.exit(1)
trans_field = drone.getField("translation")
# rotation_field = drone.getField("rotation")

while supervisor.step(timestep) != -1:
    linear_pos = trans_field.getSFVec3f()
    # angular_pos = rotation_field.getSFRotation()

    # r = m3d.Orientation.new_axis_angle([angular_pos[0], angular_pos[1], angular_pos[2]], angular_pos[3])
    # print(r)

    # print("Mavic 2 PRO is at position: %g %g %g" % (values[0], values[1], values[2]))
    supervisor.setLabel(
        0, "% .2f % .2f % .2f" % (linear_pos[0], linear_pos[2], linear_pos[1]), 0, 0.9, 0.2, 0xFF0000, 0, "Arial"
    )
    # supervisor.setLabel(
    #    1,
    #    "% .2f % .2f % .2f % .2f" % (angular_pos[0], angular_pos[1], angular_pos[2], angular_pos[3]),
    #    0,
    #    0.8,
    #    0.2,
    #    0xFF0000,
    #    0,
    #    "Arial",
    # )
