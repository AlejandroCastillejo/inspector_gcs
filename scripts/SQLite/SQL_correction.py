#!/usr/bin/env python

import rospy
import sqlite3
import csv
import json
import os.path
import uuid


class SqlCorrection:

    def __init__(self):

        # Desktop path
        self.desktop = os.path.expanduser("~/Desktop")

        # Files path
        # self.telemetry_file = self.desktop + '/mission_bags/webcam.csv/georefenencedImages.csv'
        # self.telemetry_file = self.desktop + '/georefenenced_rgb_images_2019-06-13.csv'
        # mission_file = "/home/alejandro/catkin_ws/src/inspector_gcs/json_files/archivo-mision.json"

        old_db = sqlite3.connect('example_lat&lon.db')
        new_db = sqlite3.connect('example_lat&lon_2.db')

        c_old = old_db.cursor()
        c_new = new_db.cursor()

        c_old.execute("SELECT name FROM sqlite_master WHERE type='table'")

        # with open(mission_file) as f:
        #     json_file = json.load(f)
        #     devices = json_file['devices']
        # print ('devices:')
        # print (devices)
        # devices_uuids = {}
        # for device in devices:
        #     model = device["model"]
        #     model_variables = {}
        #     for variable in device["variables_to_read"]:
        #         if variable["name"] == "dji sdk gps pos lat":
        #             model_variables.update({"lat_uuid" : variable["uuid"]})
        #         if variable["name"] == "dji sdk gps pos lon":
        #             model_variables.update({"lon_uuid" : variable["uuid"]})
        #     devices_uuids.update({model : model_variables})
                    
        # print ('\ndevices_uuids: ') 
        # print(devices_uuids)

        # SQL CREATION SENSOR ENTRIES:
        c_new.execute('''
        CREATE TABLE IF NOT EXISTS sensor_entries (id INTEGER, uuid TEXT NOT NULL, timestamp REAL NOT NULL, value REAL NOT NULL, PRIMARY KEY (id))
        ''')

        new_db.commit()

        rows = c_old.fetchall()

        for row in rows:
            print row[0]
            print row[1]

        print rows
        # print data

        # SQL INSERT LAST SENSOR ENTRIES:
        # c.execute("INSERT INTO last_sensor_entries(uuid,timestamp,value) VALUES(?,?,?)",(str(uuid1), 564, 644))
        # db.commit()

        # FILL telemetry_entries FROM telemetry.csv FILE
        # with open(self.telemetry_file) as csv_file:
        #     csv_reader = csv.reader(csv_file)
        #     line_count = 0
        #     for row in csv_reader:
        #         if line_count == 0:
        #             line_count += 1
        #         else:
        #             # print(row)
        #             c.execute(''' 
        #             INSERT INTO sensor_entries(uuid, timestamp, value) VALUES(?,?,?) ''', 
        #             (devices_uuids["DJI SDK UAV 1"]["lat_uuid"], row[1], row[2]) )
        #             c.execute(''' 
        #             INSERT INTO sensor_entries(uuid, timestamp, value) VALUES(?,?,?) ''', 
        #             (devices_uuids["DJI SDK UAV 1"]["lon_uuid"], row[1], row[3]) )
        #             # c.execute(''' 
        #             # INSERT INTO telemetry_entries(latitude, longitude, altitude) VALUES(?,?,?) ''', 
        #             # (row[2], row[3], row[4]) )
        #             # c.execute(''' 
        #             # INSERT INTO geotaged_images(uuid, timestamp, image_file, latitude, longitude, altitude) VALUES(?,?,?,?,?,?) ''',
        #             # (str(uuid.uuid1()), row[1], row[0], row[2], row[3], row[4]) )
        #     db.commit()
            








def main():
    # rospy.init_node('SqlManager')
    sql_correction = SqlCorrection()
    # rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
