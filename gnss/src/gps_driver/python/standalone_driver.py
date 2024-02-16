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

def lat_degMinstoDec(latitude):
    deg = int(latitude[:2])
    mins = float(latitude[2:])
    degDec = mins / 60
    return deg + degDec

def long_degMinstoDec(longitude):
    if len(longitude)>9:
        deg = int(longitude[:3])
        mins = float(longitude[3:])
    else:
        deg = int(longitude[:2])
        mins = float(longitude[2:])
    degDec = mins / 60
    return deg + degDec

def LatLongSign(LatLong, LatLongDir):
   if LatLongDir == "S" or LatLongDir =="W" :
       LatLong *= -1
   return LatLong


def convertToUTM(LatitudeSigned, LongitudeSigned):
    UTM_Vals = utm.from_latlon(float(LatitudeSigned), float(LongitudeSigned))
    UTM_Easting, UTM_Northing, UTM_Zone, UTM_Letter = UTM_Vals
    print(f'UTMEasting: {UTM_Easting}')
    print(f'UTMNorthing: {UTM_Northing}')
    print(f'UTMZone: {UTM_Zone}')
    print(f'UTMLetter: {UTM_Letter}')
    return [UTM_Easting, UTM_Northing, UTM_Zone, UTM_Letter]

def UTCtoUTCEpoch(UTC):
   
        UTC = float(UTC)

        UTCinSecs = (UTC // 10000) * 3600 + ((UTC % 10000) // 100) * 60 + (UTC % 100)
        Current_Time = datetime.utcnow()
        Current_UTC_Time = datetime.utcnow()
        Midnight_UTC = datetime.combine(Current_UTC_Time, datetime.min.time(), tzinfo=timezone.utc)    
        TimeSinceEpochBOD = Midnight_UTC.timestamp()
        Current_Time_Sec = (Current_Time)
        Current_Time_Nsec = ((Current_Time - Current_Time_Sec) * 1e9)
        return [Current_Time_Sec, Current_Time_Nsec]

if __name__ == '__main__':
    rospy.init_node('gps_driver')
    serialPortAddr = rospy.get_param('~port', '/dev/ttyUSB0')
    serialPort = serial.Serial(serialPortAddr, 4800)
    pub =rospy.Publisher('gps',Customgps,queue_size=1) 
    bag = rosbag.Bag('output.bag', 'w')
    try:
        while not rospy.is_shutdown():
            gpgga_Read = str(serialPort.readline())
            if "$GPGGA" in gpgga_Read:
                # pub =rospy.Publisher('gps',Customgps,queue_size=5)
                gpggaSplit = gpgga_Read.split(',')
                print(f'the GPGGA string is:{gpggaSplit}')
                UTC = gpggaSplit[1]
                Latitude = str(gpggaSplit[2])
                LatitudeDir = str(gpggaSplit[3])
                Longitude = str(gpggaSplit[4])
                LongitudeDir = str(gpggaSplit[5])
                Altitude = float(gpggaSplit[9])
                HDOP =float (gpggaSplit[8])
                print(f'UTC: {UTC}')
                print(f'Latitude: {Latitude}')
                print(f'LatitudeDir: {LatitudeDir}')
                print(f'Longitude: {Longitude}')
                print(f'LongitudeDir: {LongitudeDir}')
                print(f'HDOP: {HDOP}')
                LatitudeDec = lat_degMinstoDec(Latitude)
                LongitudeDec = long_degMinstoDec(Longitude)
                LatitudeSigned = LatLongSign(LatitudeDec, LatitudeDir)
                LongitudeSigned = LatLongSign(LongitudeDec, LongitudeDir)
                print(f'Latitude: {LatitudeSigned}')
                print(f'Longitude: {LongitudeSigned}')
                UTM_Vals = convertToUTM(LatitudeSigned, LongitudeSigned)    
                Current_Time = UTCtoUTCEpoch(UTC)
                print(f'CurrentTime: {Current_Time}')                
                rospy.loginfo('publishing latitude,longitude,UTM data')
                cgps_msg = Customgps()
                cgps_msg.header.frame_id = 'GPS1_Frame'
                cgps_msg.header.stamp = rospy.Time(int(time.time()), int((time.time() % 1) * 1e9))
                cgps_msg.latitude = Latitude
                cgps_msg.longitude = Longitude
                cgps_msg.altitude = Altitude
                # UTMVals = utm.from_latlon((LatitudeSigned),(LongitudeSigned))
                cgps_msg.utm_easting = UTM_Vals[0]
                cgps_msg.utm_northing = UTM_Vals[1]
                cgps_msg.zone = int(UTM_Vals[2])
                cgps_msg.letter = UTM_Vals[3]
                cgps_msg.hdop = HDOP
                cgps_msg.gpgga_read=gpgga_Read
                cgps_msg.header.stamp = rospy.Time.now()
                pub.publish(cgps_msg)
                bag.write('gps',cgps_msg)

               
            rospy.sleep(1)
        bag.close()

    except rospy.ROSInterruptException:
        serialPort.close()
       
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down paro_depth node...")
