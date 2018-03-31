import json


def is_json(data):
   try:
      json_object = json.loads(data)
   except ValueError, e:
      return False
   return True

def stop_or_start_parse(data):
   
   print("*******************************", data, "****************************************")

   parsed_json = json.loads(data)
   action = parsed_json['action']
   robot_id = parsed_json['robot_id']

   return str(action), int(robot_id)


def start_mission_parse(data):

   parsed_json = json.loads(data)
   mission_id = parsed_json['missionId']
   message = "mission" + str(mission_id)

   return message
