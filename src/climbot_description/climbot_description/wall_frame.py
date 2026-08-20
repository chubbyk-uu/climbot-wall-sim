"""Fixed transform between the Gazebo world frame and the wall work frame."""

# Every node that converts Gazebo truth into wall coordinates loads the same
# description, so the wall can be moved or re-oriented without editing code.

from climbot_description.geometry import (
    quaternion_conjugate,
    quaternion_from_rpy,
    quaternion_multiply,
    rotate_vector,
)
import yaml


class WallFrame:
    """Right-handed wall work frame: +X along the wall, +Y up, +Z outward."""

    # The origin is the wall's lower-left corner, so the working surface is
    # x in [0, width] and y in [0, height] and no wall coordinate is negative.
    #
    # The stored pose is that of the wall frame expressed in the Gazebo world
    # frame, so position_from_world maps world coordinates into wall ones.

    def __init__(self, origin_xyz, origin_rpy, surface=None):
        if len(origin_xyz) != 3 or len(origin_rpy) != 3:
            raise ValueError('Wall origin_xyz and origin_rpy need three values each.')
        self.origin = tuple(float(value) for value in origin_xyz)
        self.roll_pitch_yaw = tuple(float(value) for value in origin_rpy)
        self.surface = dict(surface or {})
        self._world_from_wall = quaternion_from_rpy(*self.roll_pitch_yaw)
        self._wall_from_world = quaternion_conjugate(self._world_from_wall)

    @classmethod
    def from_yaml(cls, path):
        """Load a wall frame from a YAML file with a top-level `wall` key."""
        with open(path) as handle:
            document = yaml.safe_load(handle)
        if not isinstance(document, dict) or 'wall' not in document:
            raise ValueError('%s has no top-level "wall" section.' % path)
        wall = document['wall']
        for key in ('origin_xyz', 'origin_rpy'):
            if key not in wall:
                raise ValueError('%s is missing wall.%s.' % (path, key))
        return cls(wall['origin_xyz'], wall['origin_rpy'], wall.get('surface'))

    def position_from_world(self, position):
        """Return an (x, y, z) Gazebo world position in wall coordinates."""
        offset = (
            position[0] - self.origin[0],
            position[1] - self.origin[1],
            position[2] - self.origin[2],
        )
        return rotate_vector(self._wall_from_world, offset)

    def orientation_from_world(self, quaternion):
        """Return an (x, y, z, w) world orientation in wall coordinates."""
        return quaternion_multiply(self._wall_from_world, quaternion)

    @property
    def rotation_world_from_wall(self):
        """Return the (x, y, z, w) rotation of the wall frame in the world."""
        return self._world_from_wall
