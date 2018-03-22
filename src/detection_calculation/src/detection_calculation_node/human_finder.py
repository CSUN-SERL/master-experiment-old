
import tf
import math
import numpy

import wall_rtree
import human_rtree
from util import *


class HumanFinder:
    def __init__(self, walls_dict, humans_dict, depth_of_field, field_of_view, field_of_view_margin):
        self.walls_dict = walls_dict
        self.humans_dict = humans_dict

        self.depth_of_field = depth_of_field
        self.field_of_view = field_of_view
        self.field_of_view_margin = field_of_view_margin

        self.wall_spatial_indexer = wall_rtree.WallRtree(walls_dict)
        self.human_spatial_indexer = human_rtree.HumanRTree(humans_dict)

    def get_robot_bounding_box(self, robot_x, robot_y, depth_of_field):
        left = robot_x - depth_of_field
        bottom = robot_y - depth_of_field
        right = robot_x + depth_of_field
        top = robot_y + depth_of_field

        return left, bottom, right, top

    def field_of_view_filter(self, robot_x, robot_y, robot_angle, field_of_view, depth_of_field, humans_dict):
        fov_filtered_humans_dict = {}
        # for id, human_x, human_y, dclass in humans_dict:
        for id, human_data in humans_dict:

            # Get the coordinates of the human relative to the robot
            hx, hy = shift_points(robot_x, robot_y, human_data['x'], human_data['y'])

            # Get the distance between the humann and the robot
            dist = cartesian_to_polar_distance(hx, hy)

            if dist <= depth_of_field:  # dof
                # Rotoate the human and the robot so the robot is facing X=0, and get the human's new coordinates
                relHX, relHY = normalize_to_angle(hx, hy, robot_angle)

                # Get the new angle between the robot and the human
                human_angle = cartesian_to_polar_angle(relHX, relHY)

                human_data['human_angle'] = human_angle

                human_data['distance_to_robot'] = dist

                fov_offset = (field_of_view/ 2.0) - self.field_of_view_margin

                # If the human has an angle less than half the robot's FOV (fov_offset)...
                if (human_angle <= fov_offset) and (human_angle >= (fov_offset * -1)) and human_data['dclass'] != 2:
                    fov_filtered_humans_dict[id] = human_data

        return fov_filtered_humans_dict

    def find_people_in_view(self, robot_x, robot_y, robot_angle):
        left, bottom, right, top = self.get_robot_bounding_box(robot_x, robot_y, self.depth_of_field)
        nearby_walls = self.wall_spatial_indexer.search(left, bottom, right, top)
        nearby_humans = self.human_spatial_indexer.search(left, bottom, right, top)
        nearby_humans_in_fov = self.field_of_view_filter(nearby_humans, robot_x, robot_y,
                                                         robot_angle, self.depth_of_field)

        humans_seen_by_camera = {}

        for human_id, human_data in nearby_humans_in_fov.iteritems():
            for wall_index in nearby_walls:
                wall = self.walls_dict[wall_index]
                yaw = wall['yaw']

                if yaw == 0:
                    tl = wall['p3']
                    br = wall['p2']

                if yaw == 1.5708:
                    tl = wall['p4']
                    br = wall['p1']

                if yaw == -1.5708:
                    tl = wall['p1']
                    br = wall['p4']

                if yaw == 3.14159:
                    tl = wall['p2']
                    br = wall['p3']

                if self.wall_intersects_view_to_human(human_data['x'], human_data['y'], tl, br, robot_x, robot_y):
                    break

            humans_seen_by_camera[human_id] = human_data

        return humans_seen_by_camera

    def _on_segment(self, p, q, r):
        if q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]):
            return True

        return False

    def _find_orientation(self, p, q, r):
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if val == 0:
            return 0
        if val > 0:
            return 1
        return 2

    def _lines_intersect(self, a1, a2, b1, b2):
        o1 = self._find_orientation(a1, a2, b1)
        o2 = self._find_orientation(a1, a2, b2)
        o3 = self._find_orientation(b1, b2, a1)
        o4 = self._find_orientation(b1, b2, a2)

        if o1 != o2 and o3 != o4:
            return True

        if o1 == 0 and (self._on_segment(a1, b1, a2) or self._on_segment(a1, b2, a2) or self._on_segment(b1, a1, b2)
                        or self._on_segment(b1, a2, b2)):
            return True

        return False

    def wall_intersects_view_to_human(self, human_x, human_y, wall_tl, wall_br, robot_x, robot_y):
        #Get wall diagonals
        wall_tr = (wall_br[0], wall_tl[1])
        wall_bl = (wall_tl[0], wall_br[1])
        wall_diag_1 = (wall_tl, wall_br)
        wall_diag_2 = (wall_tr, wall_bl)
        wall_diags = [wall_diag_1, wall_diag_2]
        human_line = ((robot_x, robot_y), (human_x, human_y))
        #For each wall diagonal
        for wall_diag in wall_diags:
            if self._lines_intersect(human_line[0], human_line[1], wall_diag[0], wall_diag[1]):
                return True
        return False
