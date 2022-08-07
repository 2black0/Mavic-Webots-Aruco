"""mavic_supervisor controller."""

from controller import Supervisor
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
import math3d as m3d

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

drone = supervisor.getFromDef("drone")
if drone is None:
    sys.stderr.write("No DEF supervisor node found in the current world file\n")
    sys.exit(1)

while supervisor.step(timestep) != -1:
    pos = np.array(drone.getPosition())
    # print(robot_pos)
    supervisor.setLabel(0, "% .2f % .2f % .2f" % (pos[0], pos[2], pos[1]), 0, 0.9, 0.2, 0xFF0000, 0, "Arial")

    robot_rot = np.transpose(np.array(drone.getOrientation()).reshape(3, 3))
    # robot_rot = np.array(drone.getOrientation()).reshape(3, 3)
    # print(robot_rot)

    r = R.from_matrix([robot_rot[0], robot_rot[1], robot_rot[2]])
    deg = r.as_euler("yxz", degrees=True)
    # deg = r.as_rotvec()
    print("rol_={: .2f}|pitc_={: .2f}|ya_={: .2f}".format(deg[0], deg[1], deg[2]))
    # supervisor.setLabel(0, "% .2f % .2f % .2f" % (deg[0], deg[1], deg[2]), 0, 0.9, 0.2, 0xFF0000, 0, "Arial")

    robot_pose = np.array(drone.getPose()).reshape(4, 4)
    # print(robot_pose)
    # x = robot_pose[3]
    # z = robot_pose[7]
    # y = robot_pose[11]
    # print("xpos={: .2f}|ypos={: .2f}|zpos={: .2f}".format(x, y, z))
