
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
        fov_filtered_humans_dict = []
        for id, human_x, human_y, dclass in humans_dict:

            # Get the coordinates of the human relative to the robot
            hx, hy = shift_points(robot_x, robot_y, human_x, human_y)

            # Get the distance between the humann and the robot
            dist = cartesian_to_polar_distance(hx, hy)

            if dist <= depth_of_field:  # dof
                # Rotoate the human and the robot so the robot is facing X=0, and get the human's new coordinates
                relHX, relHY = normalize_to_angle(hx, hy, robot_angle)

                # Get the new angle between the robot and the human
                human_angle = cartesian_to_polar_angle(relHX, relHY)

                fov_offset = (field_of_view/ 2.0) - self.field_of_view_margin

                # If the human has an angle less than half the robot's FOV (fov_offset)...
                if (human_angle <= fov_offset) and (human_angle >= (fov_offset * -1)) and dclass != 2:
                    fov_filtered_humans_dict[id] = humans_dict[id]

        return fov_filtered_humans_dict

    def find_people_in_view(self, robot_x, robot_y, robot_angle):
        left, bottom, right, top = self.get_robot_bounding_box(robot_x, robot_y, self.depth_of_field)
        nearby_walls = self.wall_spatial_indexer.search(left, bottom, right, top)
        nearby_humans = self.human_spatial_indexer.search(left, bottom, right, top)
        nearby_humans_in_fov = self.field_of_view_filter(nearby_humans, robot_x, robot_y,
                                                         robot_angle, self.depth_of_field)
        for human in nearby_humans_in_fov:
            for wall in nearby_walls:
                break