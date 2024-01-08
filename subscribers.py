#!/Library/Frameworks/Python.framework/Versions/3.12/bin/python3

import rospy
import json
import rosbag
import math
import numpy as np
import pandas as pd
from datetime import datetime
from sensor_msgs.msg import NavSatFix

# Add or change topics here
gps_head_data = '/tric_navigation/gps/head_data'
gps_tail_data = '/tric_navigation/gps/tail_data'
#Add or chnage paths here
json_map_path = 'json_maps/testrow4.json'
bag_path = 'e0_rosbags/2023-12-06-15-32-37.bag'

"""JSONData is a class that returns dataframes for the start path, end path, rows, turns, datum, and wing boom position."""
class JSONData:

    def __init__(self):
        global json_map_path
        with open(json_map_path, "r") as json_file:
            self.json_data = json.load(json_file)

        self.validate_json_structure()

    def validate_json_structure(self):
        if 'points' not in self.json_data or 'datum' not in self.json_data:
            raise ValueError("Invalid JSON structure. Missing 'points' or 'datum'.")
    
    def extract_data(self):
        treatment_area_indices = [i for i, point in enumerate(self.json_data['points']) if point.get('treatment_area', False)]
        first_treatment_area_index = next((i for i, point in enumerate(self.json_data['points']) if point.get('treatment_area', False)), None)
        last_point_index = len(self.json_data['points']) - 1
        last_treatment_area_index = next((i for i in range(last_point_index, -1, -1) if self.json_data['points'][i].get('treatment_area', False)), None)

        data = {
            'rows': [
                {
                    'x': (point['head']['position']['x']),
                    'y': (point['head']['position']['y'])
                }
                for point in self.json_data['points'] if point.get('treatment_area', False)
            ],
            'turns': [
                {
                    'x': (point['head']['position']['x']),
                    'y': (point['head']['position']['y'])
                }
                for i in range(1, len(treatment_area_indices))
                for point in self.json_data['points'][treatment_area_indices[i - 1] + 1: treatment_area_indices[i]]
            ],
            'start_path': [
                {
                    'x': (point['head']['position']['x']),
                    'y': (point['head']['position']['y'])
                }
                for point in self.json_data['points'][:first_treatment_area_index + 1]
            ],
            'end_path': [
                {
                    'x': (point['head']['position']['x']),
                    'y': (point['head']['position']['y'])
                }
                for point in self.json_data['points'][last_treatment_area_index + 1:]
            ],
        }

        return data 

class GPSData:
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.datum_longitude = JSONData().json_data['datum']['longitude']
        self.datum_latitude = JSONData().json_data['datum']['latitude']
        self.data = []
        rospy.init_node('gps_data_node', anonymous=True)
        rospy.Subscriber(self.topic_name, NavSatFix, self.callback)
        rospy.spin()

    def callback(self, msg):
        x_m, y_m = self.gps_to_meters(msg.longitude, msg.latitude)
        t = rospy.get_time()
        if self.topic_name == '/tric_navigation/gps/tail_data':
            self.data.append({'timestamp': t, 'tail_x_m': x_m, 'tail_y_m': y_m})
        else:
            self.data.append({'timestamp': t, 'head_x_m': x_m, 'head_y_m': y_m})

    def gps_to_meters(self, lon2, lat2):
        lon1 = self.datum_longitude
        lat1 = self.datum_latitude
        R = 6371  # radius of earth at equator (km)
        alpha1 = lat1  # alpha (deg)
        r1 = R * math.cos(math.pi * alpha1/180) 
        l1 = r1 * math.pi/180
        L1 = R * math.pi/180

        alpha2 = lat2  # alpha (deg)
        r2 = R * math.cos(math.pi * alpha2/180) 
        l2 = r2 * math.pi/180
        L2 = R * math.pi/180

        x_km = L1 * (lat2 - lat1)
        y_km = l1 * (lon2 - lon1)
        x_m = x_km * 1000  # convert km to meters
        y_m = y_km * 1000

        return x_m, y_m

def main():

    print(GPSData(gps_head_data, bag_path).to_dataframe())
    
if __name__ == '__main__':
    main()
