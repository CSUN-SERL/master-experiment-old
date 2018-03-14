import json
from decimal import Decimal as unicode_convert
import yaml
import math
import numpy as np


class floor:
    def concat_walls(self,json_data):
        wall_arr = []
        # pass the floor centroid data to correct the
        # subwall data
        floor_data = self
        for i in json_data:
            wall_arr.append(wall(i,floor_data))
        return wall_arr

    def __init__(self,json_data):
        self.centroid_x = json_data['px']
        self.centroid_y = json_data['py']
        self.centroid_z = json_data['pz']
        self.walls = self.concat_walls(json_data['links'])


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
            cos , sin = np.cos(self.yaw), np.sin(self.yaw)
            rotation_matrix = np.array(((cos,-sin),(sin,cos)))

            centroid = centroid_x + subwall_data['px'] , centroid_y + subwall_data['py']

            #set intial values of p1,p2,p3,p4
            self.p1 = centroid[0] - subwall_data['gx']/2 , centroid[1] - subwall_data['gy']/2
            self.p2 = centroid[0] + subwall_data['gx']/2 , centroid[1] - subwall_data['gy']/2
            self.p3 = centroid[0] - subwall_data['gx']/2 , centroid[1] + subwall_data['gy']/2
            self.p4 = centroid[0] + subwall_data['gx']/2 , centroid[1] + subwall_data['gy']/2

            #rotate the points
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


        def get_min_max(self):
            point_arr_x = [self.p1[0],self.p2[0],self.p3[0],self.p4[0]]
            point_arr_y = [self.p1[1],self.p2[1],self.p3[1],self.p4[1]]

            return min(point_arr_x),min(point_arr_y),max(point_arr_x),max(point_arr_y)




    def convert_to_border(self,json_data):
        subwalls_original = json_data['subwalls']

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

    def __init__(self,json_data,floor_data):

        #pass the
        self.centroid_x = float(json_data['px']) + float(floor_data.centroid_x)
        self.centroid_y = float(json_data['py']) + float(floor_data.centroid_y)
        self.centroid_z = float(json_data['pz']) + float(floor_data.centroid_z)
        self.centroid_yaw = float(json_data['yaw'])
        self.subwalls = self.convert_to_border(json_data)


    def get(self,index):
        return subwalls[index]


if __name__ == "__main__":
    json_walls = open('walls.json').read()
    wall_data = json.loads(json_walls)

    # converting subwal position relative to centroid
    # to the global coordinate system in gazebo
    my_floors = []
    for a_floor in wall_data:
        my_floors.append(floor(a_floor))



    yaml_file = open('walls.yaml','w+')
    count = 0

    min_x = 0
    min_y = 0
    max_x = 0
    max_y = 0
    for a_floor in my_floors:
        for centroid in a_floor.walls:
            for subwall in centroid.subwalls:
                yaml_file.write(yaml.dump({ count :{'yaw' : subwall.yaw,
                                                    'p1' : [subwall.p1[0],subwall.p1[1]],
                                                    'p2' : [subwall.p2[0],subwall.p2[1]],
                                                    'p3' : [subwall.p3[0],subwall.p3[1]],
                                                    'p4' : [subwall.p4[0],subwall.p4[1]],
                                                    }}))
                this_min_x, this_min_y,this_max_x,this_max_y = subwall.get_min_max()

                if min_x > this_min_x:
                    min_x = this_min_x
                if min_y > this_min_y:
                    min_y = this_min_y
                if max_x < this_max_x:
                    max_x = this_max_x
                if max_y < this_max_y:
                    max_y = this_max_y

                count +=1

    yaml_file.write(yaml.dump({ 'stats' :{'min_x',}}))# :{'min_x' : min_x ,
                                        #    'max_x' : max_x ,
                                        #    'min_y' : min_y ,
                                        #    'max_y' : max_y ,
                                        #    }}))
