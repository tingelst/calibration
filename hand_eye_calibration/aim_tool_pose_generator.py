import math3d as m3d
import numpy as np

class AimToolPoseGenerator(object):
    """ Generator for random tool moves, fulfilling a constraint for
    aiming the tool approach towards a target."""
    def __init__(self, centre_pos=m3d.Vector([1.12, 0.0, 1.27]),
                 aim_pos=m3d.Vector([2.2, 0.2, 0.8]),
                 tool_roll_centre=np.deg2rad(156),
                 tool_roll_var=np.pi/6,
                 tool_pos_var = 0.2):
        self._min_move = 0.1
        self._aim_pos = aim_pos
        self._tool_roll_centre = tool_roll_centre
        self._tool_roll_var = tool_roll_var
        self._tool_pos_var = tool_pos_var
        if not centre_pos is None:
            self.centre_pos = centre_pos
        else:
            self._centre_pos = None

    @property
    def centre_pos(self):
        return self._centre_pos

    @centre_pos.setter
    def centre_pos(self, centre_pos):
        self._centre_pos = centre_pos
        cp = centre_pos
        bl = self._tool_pos_var
        # // Limit box side length
        self._x_lims = (cp.x - bl, cp.x + bl)
        self._y_lims = (cp.y - bl, cp.y + bl)
        self._z_lims = (cp.z - bl, cp.z + bl)
        #self._tool_rotation_angle = 0
        self._tool_roll_lims = np.array((-self._tool_roll_var, self._tool_roll_var)) \
                + self._tool_roll_centre

    def gen_pose(self):
        """ Generate a new pose within the prismatic tool position
        limits, aiming at aim_pos, and obeying approach rotation
        limits."""
        gen_pos = m3d.Vector(
            np.random.uniform(*self._x_lims),
            np.random.uniform(*self._y_lims),
            np.random.uniform(*self._z_lims))
        tool_dir_z = (self._aim_pos - gen_pos).normalized()
        tool_dir_y = m3d.Vector.e2.copy()
        rot_y = m3d.Orientation()
        tool_dir_y -= ((tool_dir_y * tool_dir_z) * tool_dir_z)
        rot_y.axis_angle = [tool_dir_z, np.random.uniform(*self._tool_roll_lims)]
        tool_dir_y = (rot_y * tool_dir_y).normalized()
        tool_dir_x = tool_dir_y.cross(tool_dir_z)
        gen_orient = m3d.Orientation(tool_dir_x, tool_dir_y, tool_dir_z)
        return m3d.Transform(gen_orient, gen_pos)

    __call__ = gen_pose

