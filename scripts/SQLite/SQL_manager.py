#!/usr/bin/env python

import rospy
import sqlite3
import csv
import os.path
import uuid


class SqlManager:

    def __init__(self):

        # Desktop path
        self.desktop = os.path.expanduser("~/Desktop")

        # Files path
        self.telemetry_file = self.desktop + '/mission_bags/webcam.csv/georefenencedImages.csv'

        db = sqlite3.connect('example.db')

        c = db.cursor()

        # SQL CREATION SENSOR ENTRIES:
        c.execute("CREATE TABLE IF NOT EXISTS sensor_entries (id INTEGER, uuid TEXT NOT NULL, timestamp REAL NOT NULL, value REAL NOT NULL, PRIMARY KEY (id))")

        # SQL CREATION LAST SENSOR ENTRIES:
        c.execute("CREATE TABLE IF NOT EXISTS last_sensor_entries (uuid TEXT NOT NULL, timestamp REAL NOT NULL, value REAL NOT NULL, PRIMARY KEY (uuid));")

        # SQL CREATION TELEMETRY ENTRIES:
        c.execute('''
        CREATE TABLE IF NOT EXISTS telemetry_entries (id INTEGER, uuid TEXT NOT NULL, timestamp REAL NOT NULL, 
        image_file REAL NOT NULL, latitude REAL NOT NULL, longitude REAL NOT NULL, altitude REAL NOT NULL, PRIMARY KEY (id))
        ''')

        # SQL TRIGGER:
        c.execute('''
        CREATE TRIGGER IF NOT EXISTS delete_tail AFTER INSERT ON sensor_entries 
        BEGIN DELETE FROM sensor_entries WHERE (id % 10000000 = NEW.id % 10000000) AND (id != NEW.id); END;
        ''')

        # SQL INSERT SENSOR ENTRIES:
        # uuid1 = uuid.uuid1()
        # print('uuid: ', uuid1)
        # print('uuid int: ', str(uuid1))
        # c.execute("INSERT INTO sensor_entries(uuid,timestamp,value) VALUES(?,?,?)",(str(uuid1), 51516, 51515))
        # # c.execute("INSERT INTO sensor_entries(uuid,timestamp,value) VALUES(21,23151,51516)")
        # db.commit()

        # SQL INSERT LAST SENSOR ENTRIES:
        # c.execute("INSERT INTO last_sensor_entries(uuid,timestamp,value) VALUES(?,?,?)",(str(uuid1), 564, 644))
        # db.commit()

        # FILL telemetry_entries FROM telemetry.csv FILE
        with open(self.telemetry_file) as csv_file:
            csv_reader = csv.reader(csv_file)
            line_count = 0
            for row in csv_reader:
                if line_count == 0:
                    line_count += 1
                else:
                    # print(row)
                    c.execute(''' 
                    INSERT INTO telemetry_entries(uuid, timestamp, image_file, latitude, longitude, altitude) VALUES(?,?,?,?,?,?) ''',
                    (str(uuid.uuid1()), row[1], row[5], row[6], row[7], row[8]) )
            db.commit()
            








def main():
    # rospy.init_node('SqlManager')
    sql_manager = SqlManager()
    # rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
