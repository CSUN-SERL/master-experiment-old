import json
from decimal import Decimal as unicode_convert
import yaml
import numpy as np




class wall:

    #holds global position of subwall
    class subwall:
        # p1 bottom left
        # p2 bottom right
        # p3 top left
        # p4 top right
        def __init__(self):
            self.yaw = 0
            self.p1 = []
            self.p2 = []
            self.p3 = []
            self.p4 = []

        #convert from subwall coordinates that are relative to parent centroid
        #to global goordinates relative to global origin in Gazebo
        def relative_coord_to_global_coord(self,subwall_data,wall):

            if float(subwall_data['pz']) == 2.25:
                #print 'fail'
                return False

            centroid_yaw = wall.centroid_yaw
            centroid_x = wall.centroid_x
            centroid_y = wall.centroid_y
            centroid_z = wall.centroid_z

            for i in subwall_data:
                 subwall_data[str(i)] = float(subwall_data[str(i)])

            centroid_yaw = float(wall.centroid_yaw)
            self.yaw = centroid_yaw

            subwall_yaw = np.arctan2(subwall_data['py'],subwall_data['py'])
            cos , sin = np.cos(subwall_yaw), np.sin(subwall_yaw)
            rotation_matrix = np.array(((cos,-sin),(sin,cos)))

            centroid = centroid_x + subwall_data['px'] , centroid_y + subwall_data['py']

            self.p1 = centroid[0] - subwall_data['gx']/2 , centroid[1] - subwall_data['gy']/2
            self.p2 = centroid[0] + subwall_data['gx']/2 , centroid[1] - subwall_data['gy']/2
            self.p3 = centroid[0] - subwall_data['gx']/2 , centroid[1] + subwall_data['gy']/2
            self.p4 = centroid[0] + subwall_data['gx']/2 , centroid[1] + subwall_data['gy']/2

            self.p1 = np.dot(rotation_matrix,self.p1)
            self.p2 = np.dot(rotation_matrix,self.p2)
            self.p3 = np.dot(rotation_matrix,self.p3)
            self.p4 = np.dot(rotation_matrix,self.p4)


        def set(self,subwall_data,wall):
            if self.relative_coord_to_global_coord(subwall_data,wall) != False:
                self.p1 = round(self.p1[0],4),round(self.p1[1],4)
                self.p2 = round(self.p2[0],4),round(self.p2[1],4)
                self.p3 = round(self.p3[0],4),round(self.p3[1],4)
                self.p4 = round(self.p4[0],4),round(self.p4[1],4)
            else:
                return False



    def convert_to_border(self,jdata):
        subwalls_original = jdata['subwalls']

        #gz, gy, gz are the lengths of the entire sides
        #px,py,pz are relative to the parent centroid
        #need to get global subwall_dataition of subwalls
        subwalls = []
        for i in range(len(subwalls_original)):
            pre_append = self.subwall()
            wall = self
            if pre_append.set(subwalls_original[i],wall) != False:
            #if pre_append.set(subwalls_original[i],self.centroid_x,self.centroid_y,self.centroid_z,self.centroid_yaw) != False:
                subwalls.append(pre_append)


        return subwalls

    def __init__(self,json_data):
        self.centroid_x = float(json_data['px'])
        self.centroid_y = float(json_data['py'])
        self.centroid_z = float(json_data['pz'])
        self.centroid_yaw = float(json_data['yaw'])
        self.subwalls = self.convert_to_border(json_data)


    def get(self,index):
        return subwalls[index]


if __name__ == "__main__":
    json_walls = open('walls.json').read()
    wall_data = json.loads(json_walls)

    # converting subwal position relative to centroid
    # to the global coordinate system in gazebo
    walls = []
    for i in range(len(wall_data)):
        #print 'wall : ', i
        walls.append(wall(wall_data[i]))

    # converting python object information into
    # yaml file
    count = 0
    yaml_file = open('walls.yaml','w+')
    for i in range(len(walls)):
        for j in range(len(walls[i].subwalls)):
            subs = walls[i].subwalls[j]

            yaml_file.write(yaml.dump({ str(count) : {'yaw' : subs.yaw,
                                                      'p1' : [subs.p1[0],subs.p1[1]],
                                                      'p2' : [subs.p2[0],subs.p2[1]],
                                                      'p3' : [subs.p3[0],subs.p3[1]],
                                                      'p4' : [subs.p4[0],subs.p4[1]],
                                                      }}))
            count +=1
