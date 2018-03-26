
# coding: utf-8

# In[1]:


import xml.etree.ElementTree as ElementTree


# In[2]:


import random


# In[3]:


import json


# In[4]:


world_file_location = 'c:/Users/logan/Documents/jd.world'


# In[5]:


e = ElementTree.parse(world_file_location)


# In[6]:


tree = e.getroot()


# In[7]:


human_stuff = {}


# In[8]:


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


# In[9]:


missed = random.sample(xrange(len(human_stuff)), 72)


# In[10]:


for num in missed:
    human_stuff[num]['dclass'] = 2


# In[11]:


a = json.dumps(human_stuff)


# In[12]:


print a

