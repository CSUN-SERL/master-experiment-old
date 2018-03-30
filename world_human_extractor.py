import xml.etree.ElementTree as ElementTree
import random
import json

world_file_location = 'c:/Users/logan/Documents/jd.world'

e = ElementTree.parse(world_file_location)

tree = e.getroot()

human_stuff = {}

i = 0
for child in tree.find('world').find('state'):
    if child.tag == 'model' and ('person' in child.attrib['name']):
        pose_info = child.find('pose').text.split(' ')
        human_stuff[i] = {}
        human_stuff[i]['x'] = float(pose_info[0])
        human_stuff[i]['y'] = float(pose_info[1])
        human_stuff[i]['z'] = float(pose_info[2])
        human_stuff[i]['dclass'] = 0
        i += 1

missed = random.sample(xrange(len(human_stuff)), 72)

for num in missed:
    human_stuff[num]['dclass'] = 2

a = json.dumps(human_stuff)

print a

