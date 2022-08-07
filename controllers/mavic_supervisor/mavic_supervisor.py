"""mavic_supervisor controller."""

from controller import Supervisor
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

drone = supervisor.getFromDef("drone")
if drone is None:
    sys.stderr.write("No DEF supervisor node found in the current world file\n")
    sys.exit(1)

while supervisor.step(timestep) != -1:
    pos = np.array(drone.getPosition())
    # print(robot_pos)
    supervisor.setLabel(0, "X=% .2f Y=% .2f Z=% .2f" % (pos[0], pos[2], pos[1]), 0, 0.9, 0.2, 0xFF0000, 0, "Arial")

    r = np.array(drone.getOrientation()).reshape(3, 3)
    Rot = R.from_matrix([[r[0][0], r[0][1], r[0][2]], [r[1][0], r[1][1], r[1][2]], [r[2][0], r[2][1], r[2][2]]])
    rr = Rot.as_euler("zyx", degrees=True)
    # print("rolz={: .2f}|pitcz={: .2f}|yaz={: .2f}".format((rr[2] + 90.0), -rr[1], rr[0]))
    supervisor.setLabel(
        1, "R=% .2f P=% .2f Y=% .2f" % (rr[2] + 90.0, -rr[1], rr[0]), 0, 0.8, 0.2, 0xFF0000, 0, "Arial"
    )
