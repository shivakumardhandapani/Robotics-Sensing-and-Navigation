#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import utm
from gps_driver.msg import Customgps
from std_msgs.msg import Header
import math
import time
import rosbag
from datetime import datetime, timezone

def convert_degrees_minutes_to_decimal(degrees_minutes):
    deg = int(degrees_minutes // 100)
    mins = (degrees_minutes % 100)
    deg_decimal = mins / 60
    return (deg + deg_decimal)

def apply_sign_conversion(coord, direction):
    if direction in ["S", "W"]:
        coord *= -1
    return coord

def convert_to_utm(latitude_signed, longitude_signed):
    utm_values = utm.from_latlon(float(latitude_signed), float(longitude_signed))
    return utm_values

def utc_to_utc_epoch(utc):
    utc = float(utc)
    utc_in_secs = (utc // 10000) * 3600 + ((utc % 10000) // 100) * 60 + (utc % 100)
    current_time = datetime.utcnow()
    current_utc_time = datetime.utcnow()
    midnight_utc = datetime.combine(current_utc_time, datetime.min.time(), tzinfo=timezone.utc)
    time_since_epoch_bod = midnight_utc.timestamp()
    current_time_sec = (current_time)
    current_time_nsec = ((current_time - current_time_sec) * 1e9)
    return [current_time_sec, current_time_nsec]

if __name__ == '__main__':
    rospy.init_node('gps_driver')
    serial_port_address = rospy.get_param('~port', '/dev/ttyUSB0')
    serial_port = serial.Serial(serial_port_address, 4800)
    publisher = rospy.Publisher('gps', Customgps, queue_size=1)
    bag = rosbag.Bag('ros.bag', 'w')

    try:
        while not rospy.is_shutdown():
            gpgga_read = str(serial_port.readline())
            if "$GPGGA" in gpgga_read:
                gpgga_split = gpgga_read.split(',')
                utc = gpgga_split[1]
                latitude = float(gpgga_split[2])
                latitude_dir = str(gpgga_split[3])
                longitude = float(gpgga_split[4])
                longitude_dir = str(gpgga_split[5])
                altitude = float(gpgga_split[9])
                hdop = float(gpgga_split[8])

                latitude_dec = convert_degrees_minutes_to_decimal(latitude)
                longitude_dec = convert_degrees_minutes_to_decimal(longitude)
                latitude_signed = apply_sign_conversion(latitude_dec, latitude_dir)
                longitude_signed = apply_sign_conversion(longitude_dec, longitude_dir)

                utm_values = convert_to_utm(latitude_signed, longitude_signed)
                current_time = utc_to_utc_epoch(utc)

                rospy.loginfo('Publishing latitude, longitude, UTM data')
                gps_msg = Customgps()
                gps_msg.header.frame_id = 'GPS1_Frame'
                gps_msg.header.stamp = rospy.Time(int(time.time()), int((time.time() % 1) * 1e9))
                gps_msg.latitude = latitude
                gps_msg.longitude = longitude
                gps_msg.altitude = altitude
                gps_msg.utm_easting = utm_values[0]
                gps_msg.utm_northing = utm_values[1]
                gps_msg.zone = int(utm_values[2])
                gps_msg.letter = utm_values[3]
                gps_msg.hdop = hdop
                gps_msg.gpgga_read = gpgga_read
                gps_msg.header.stamp = rospy.Time.now()
                publisher.publish(gps_msg)
                bag.write('gps', gps_msg)

            rospy.sleep(1)
        bag.close()

    except rospy.ROSInterruptException:
        serial_port.close()
       
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down gps_driver node...")
