import rtree


class HumanRTree:

    def search(self, left, bottom, right, top):
        return self.spatial_humans.intersection(((left, bottom, right, top)))

    def __init__(self, humans_dict):

        self.spatial_humans = rtree.index.Index()
        for id, human_data in humans_dict.iteritems():
            self.spatial_humans.insert(int(id), (human_data['x'], human_data['y'], human_data['x'], human_data['y']))