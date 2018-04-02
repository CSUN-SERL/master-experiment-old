import rtree


class WallRtree:

    def search(self, left, bottom, right, top):
        return self.spatial_walls.intersection(((left, bottom, right, top)))

    def __init__(self, walls_dict):
        self.spatial_walls = rtree.index.Index()
        for wall_id, seg_list in walls_dict.iteritems():
            # tuples. [0] = x, [1] = y
            tl = seg_list['p3']
            br = seg_list['p2']
            yaw = seg_list['yaw']

            left = tl[0]
            top = tl[1]
            right = br[0]
            bottom = br[1]

            self.spatial_walls.insert(int(wall_id), (left, bottom, right, top))
            #
            # try:
            #     if yaw == 0:
            #         self.spatial_walls.insert(int(wall_id), (left, bottom, right, top))
            #
            #     if yaw == 1.5708:
            #         self.spatial_walls.insert(int(wall_id), (top, left, bottom, right))
            #
            #     if yaw == -1.5708:
            #         self.spatial_walls.insert(int(wall_id), (bottom, right, top, left))
            #
            #     if yaw == 3.14159:
            #         self.spatial_walls.insert(int(wall_id), (right, top, left, bottom))
            #
            #     # print left, bottom, right, top
            #     # suc_count += 1
            # except:
            #     # self.spatial_walls.add(wall_id, ())
            #     print "failed on: {}".format(wall_id)
            #     exit(-1)