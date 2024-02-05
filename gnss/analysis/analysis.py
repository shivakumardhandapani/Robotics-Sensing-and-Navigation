#!/usr/bin/env python3


# Importing the necessary modules
import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


#Note: The file location of "your.bag" should be given in below


b = bagreader(r'/home/yogeshwaran127/gnss/data/outside/2.bag')
print(f'\n{b.topic_table}\n')
csvfiles = []
for t in b.topics:
    data = b.message_by_topic(t)
    csvfiles.append(data)

print(f'Length of csv file is : {len(csvfiles[0][0][0])}')
print(f'Data in csv file is:{csvfiles[0]}')

b = bagreader(r'/home/yogeshwaran127/gnss/data/straight_/staitionary.bag')
print(f'\n{b.topic_table}\n')
csvfiles = []
for t in b.topics:
    data = b.message_by_topic(t)
    csvfiles.append(data)

print(f'Length of csv file is : {len(csvfiles[0][0][0])}')
print(f'Data in csv file is:{csvfiles[0]}')

b = bagreader(r'/home/yogeshwaran127/gnss/data/moving/1.bag')
print(f'\n{b.topic_table}\n')
csvfiles = []
for t in b.topics:
    data = b.message_by_topic(t)
    csvfiles.append(data)

print(f'Length of csv file is : {len(csvfiles[0][0][0])}')
print(f'Data in csv file is:{csvfiles[0]}')


'''
The new location of the csv file will be printed in the terminal, copy the location and give it as the file location for the csv file
'''



# reading csv file 
df = pd.read_csv("/home/yogeshwaran127/gnss/data/outside/2/gps.csv")
fd = pd.read_csv("/home/yogeshwaran127/gnss/data/straight_/staitionary/gps.csv")
# print(df.head())
# print(df.columns[8])
# for i in list(df.columns[8]):
#     print(df[i].tolist())

# Extracting data from the DataFrame
utm_easting1 = fd["utm_easting"].tolist()
utm_northing1 = fd["utm_northing"].tolist()
altitude1 = fd["altitude"].tolist()
centroid_easting1 = np.mean(utm_easting1)
centroid_northing1 = np.mean(utm_northing1)
deviation_easting1= utm_easting1 - centroid_easting1
deviation_northing1= utm_northing1 - centroid_northing1

print("--------------------------")
print("Centroid Easting open:", centroid_easting1)
print("Centroid Northing open:", centroid_northing1)
print("Deviation in Easting: open", deviation_easting1)
print("Deviation in Northing open:", deviation_northing1)

# Extracting data from the DataFrame
utm_easting = df["utm_easting"].tolist()
utm_northing = df["utm_northing"].tolist()
altitude = df["altitude"].tolist()
centroid_easting = np.mean(utm_easting)
centroid_northing = np.mean(utm_northing)
deviation_easting= utm_easting - centroid_easting
deviation_northing= utm_northing - centroid_northing
print("--------------------------")
print("Centroid Easting occluded:", centroid_easting)
print("Centroid Northing occluded:", centroid_northing)
print("Deviation in Easting occluded:", deviation_easting)
print("Deviation in Northing occluded:", deviation_northing)


'''
Plotting script starts from here
'''
'''
# Plotting Stationary northing vs. easting scatterplot
plt.figure(figsize=(24, 6))
plt.subplot(1, 3, 1)
plt.title("Open Stationary Easting vs. Northing")
plt.xlabel("UTM Easting")
plt.ylabel("UTM Northing")
plt.scatter(utm_easting, utm_northing,color='red',label='open')
#plt.scatter(utm_easting1, utm_northing1,color='blue',label='occluded')

plt.subplot(1, 3, 2)
plt.title("Occluded Stationary Easting vs. Northing")
plt.xlabel("UTM Easting")
plt.ylabel("UTM Northing")
#plt.scatter(utm_easting, utm_northing,color='green',label='open')
plt.scatter(utm_easting1, utm_northing1,color='blue',label='occluded')

plt.subplot(1, 3, 3)'''
plt.figure(figsize=(24, 6))
plt.title("comparisson Stationary Easting vs. Northing")
plt.xlabel("UTM Easting")
plt.ylabel("UTM Northing")
plt.scatter(deviation_easting, deviation_northing,color='red',label='open')
plt.scatter(deviation_easting1, deviation_northing1,color='blue',label='occluded')
plt.legend()
plt.show()



# Plotting Stationary altitude plot
plt.figure(figsize=(24, 6))
plt.title("comparission Stationary Altitude")
plt.xlabel("Timestamp")
plt.ylabel("Altitude")
plt.plot(altitude,color='red',label='easting')
plt.plot(altitude1,color='blue',label='northing')
plt.legend()
plt.show()

plt.figure(figsize=(24, 6))
plt.subplot(1, 3, 1)
plt.title("Stationary open Histogram")
plt.xlabel("Altitude")
plt.ylabel("Frequency")
plt.hist(utm_easting, bins=20, color='skyblue', edgecolor='black',label='easting open')

plt.subplot(1, 3, 2)
plt.title("Stationary occluded Histogram")
plt.xlabel("Altitude")
plt.ylabel("Frequency")
plt.hist(utm_easting1, bins=20, color='skyblue', edgecolor='black',label='easting open')
plt.subplot(1, 3, 3)
plt.title("Stationary euclidean Histogram")
plt.xlabel("Altitude")
plt.ylabel("Frequency")
plt.hist(deviation_easting, bins=20, color='skyblue', edgecolor='black',label='easting open')
plt.hist(deviation_easting1 , bins=20, color='red', edgecolor='red',label='easting occluded')
plt.legend()
# Adjust layout for better visualization
plt.tight_layout()


# reading csv file 
df = pd.read_csv("/home/yogeshwaran127/gnss/data/moving/1/gps.csv")


# print(df.head())
# print(df.columns[8])
# for i in list(df.columns[8]):
#     print(df[i].tolist())

utm_easting = df["utm_easting"].tolist()
# print(utm_easting[2])
utm_northing = df["utm_northing"].tolist()
# print(utm_northing[2])
altitude = df["altitude"].tolist()





'''
Plotting script starts from here
'''

# Plotting Stationary northing vs. easting scatterplot
plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1)
plt.title("Moving Easting vs. Northing")
plt.xlabel("UTM Easting")
plt.ylabel("UTM Northing")
plt.legend()
plt.scatter(utm_easting, utm_northing)

# Plotting Stationary altitude plot
plt.subplot(1, 2, 2)
plt.title("Moving Altitude")
plt.xlabel("Timestamp")
plt.ylabel("Altitude")
plt.legend()
plt.plot(altitude)



# Adjust layout for better visualization
plt.tight_layout()

# Show all plots
plt.show()