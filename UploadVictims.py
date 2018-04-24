import argparse
import requests
import yaml

parser = argparse.ArgumentParser()
parser.add_argument("victim_file_location", help="Location of the victim file.")
parser.add_argument("--url", help="The URL of the server to upload to.", default="192.168.1.11")
parser.add_argument("--port", help="The port of the server to upload to.", default="8000")
args = parser.parse_args()

# Parse arguments.
SERVER_URL = args.url
PORT = args.port
victim_file_location = args.victim_file_location

INSERT_VICTIM_URL = "http://{0}:{1}/detection/upload-victims".format(SERVER_URL, PORT)

# Read YAML file.
with open(victim_file_location, "r") as victim_file:
    victims = yaml.load(victim_file)

# Upload all victims.
for victim_id, info in victims.iteritems():
    j = {
        'victim_id': int(victim_id),
        'confidence': info['confidence'],
        'dclass': info['dclass'],
        'lying': info['lying'],
        'x': info['x'],
        'y': info['y'],
        'z': info['z']
    }

    requests.post(INSERT_VICTIM_URL, json=j)
