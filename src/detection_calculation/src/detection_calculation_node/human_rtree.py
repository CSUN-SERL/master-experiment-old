import rtree


class HumanRTree:

    def search(self, left, bottom, right, top):
        return self.spatial_walls.intersection(((left, bottom, right, top)))

    def __init__(self, humans_dict):

        self.spatial_humans = rtree.index.Index()
        for id, x, y in humans_dict.iteritems():
            self.spatial_humans.add(id, (x, y, x, y))