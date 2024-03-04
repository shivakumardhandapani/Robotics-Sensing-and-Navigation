import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import math
from numpy.polynomial.polynomial import polyfit
from numpy.polynomial import Polynomial
import allantools as alan
import time


# def bag_to_csv(path):
#     b = bagreader(path)
#     print(f'\n{b.topic_table}\n')
#     csv_files = []
#     for t in b.topics:
#         data = b.message_by_topic(t)
#         csv_files.append(data)

#     print(f'Length of CSV file is: {len(csv_files[0][0][0])}')
#     print(f'CSV file location: {csv_files[0]}')
#     return csv_files[0]


def quaternion_to_euler(x, y, z, w):
    import math
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z


def is_VNYMR_in_string(input_string):
    input_string = input_string.strip()
    if input_string.startswith('$VNYMR'):
        print("Great Success")
        return True
    elif input_string == '$VNYMR':
        print("Exact string match")
        return True
    else:
        print('VNYMR not found in string')
        return False


def convert_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

#csv_path = r"/home/shivakumardhandapani/imu/data/LocationC.csv"
df = pd.read_csv("/home/shivakumardhandapani/data/imu/LocationC.csv")
print(f'The columns are : {df.columns}')
data_string = df["data"].tolist()
time = df["Time"].tolist()
bad_string_counter = 0
yaw_values = []
pitch_values = []
gyro_x = []
gyro_y = []
gyro_z = []

for s in range(len(data_string)):
    current_data = data_string[s]
    if not isinstance(current_data, str):
        print(f"Skipping non-string data at index {s}")
        continue

    input_string = current_data.split(',')


    if not is_VNYMR_in_string(str(input_string[0])):
        bad_string_counter += 1            
        continue

    print(f'The number of bad strings: {bad_string_counter}')
    Time = time[s]
    try:
        yaw = float(input_string[1])
        pitch = float(input_string[2])
        roll = float(input_string[3])
        magnetic_x = float(input_string[4])
        magnetic_y = float(input_string[5])
        if len(input_string) > 6:
            magnetic_z = float(input_string[6])
        else:
            magnetic_z = 0.0
        accel_x = float(input_string[7])
        accel_y = float(input_string[8])
        accel_z = float(input_string[9])
        angular_rate_x = float(input_string[10])
        angular_rate_y = float(input_string[11])
        gyro_string = str(input_string[12].split('*')[0])
        gyro_value = float(gyro_string) if gyro_string != '+' else 0.0
        angular_rate_z = gyro_value
        print(f'The string is: {input_string}\n')
    except (ValueError, IndexError) as e:
        print(f'Error processing input: {e}')
        continue

    yaw_values.append(yaw)
    pitch_values.append(pitch)
    gyro_x.append(angular_rate_x)
    gyro_y.append(angular_rate_y)
    gyro_z.append(angular_rate_z)

np_gyro_x = np.array(gyro_x)
np_gyro_y = np.array(gyro_y)
np_gyro_z = np.array(gyro_z)

print(f'Length of time : {len(time)}  and length of gyro : {len(np_gyro_x)}')

taus2_x, ad_x, ade_x, ns_x = alan.oadev(np_gyro_x, rate=40.0, data_type="phase", taus="all")
taus2_y, ad_y, ade_y, ns_y = alan.oadev(np_gyro_y, rate=40.0, data_type="phase", taus="all")
taus2_z, ad_z, ade_z, ns_z = alan.oadev(np_gyro_z, rate=40.0, data_type="phase", taus="all")

print(f'Length of ad_x : {len(ad_x)}, length of ad_y : {len(ad_y)}, length of ad_z : {len(ad_z)}')

# Plotting fig - xx
plt.title("Plotting Allan deviation")
plt.xlabel("Time in s")
plt.ylabel("Allan Deviation")
plt.loglog(taus2_x, ad_x, label="Gyro X", color="Red")
plt.loglog(taus2_y, ad_y, label="Gyro Y", color="Purple")
plt.loglog(taus2_z, ad_z, label="Gyro Z", color="Yellow")

plt.legend()
plt.show()

def extract_parameters(taus, ad):
    # Find Bias Instability (B) and its corresponding tau
    idx_B = np.argmin(ad)
    B = ad[idx_B]
    tau_B = taus[idx_B]
    
    # Estimating parameters based on slopes needs more careful selection of the linear regions
    # For this example, we use a simplified method to illustrate the process

    # Assuming that the Angle Random Walk (N) is at the smallest tau
    N = np.sqrt(ad[0] * taus[0])

    # Find the Rate Random Walk (K) by looking for a slope of -0.5
    # The slope calculation should be done by fitting the line in the appropriate region
    # For this example, we will just use the central third to illustrate
    log_taus = np.log10(taus)
    log_ads = np.log10(ad)
    mid_point = len(taus) // 3
    b, m = polyfit(log_taus[mid_point:2*mid_point], log_ads[mid_point:2*mid_point], 1)
    
    # Check if the slope is approximately -0.5
    if -0.6 < m < -0.4:
        # K is then the Allan deviation at this point
        K = np.sqrt(ad[mid_point] * taus[mid_point])
    else:
        K = None

    return B, tau_B, N, K

# Extract parameters for each axis
B_x, tau_B_x, N_x, K_x = extract_parameters(taus2_x, ad_x)
B_y, tau_B_y, N_y, K_y = extract_parameters(taus2_y, ad_y)
B_z, tau_B_z, N_z, K_z = extract_parameters(taus2_z, ad_z)

# Print the results
print("Gyro X - B:", B_x, "at tau:", tau_B_x, "N:", N_x, "K:", K_x)
print("Gyro Y - B:", B_y, "at tau:", tau_B_y, "N:", N_y, "K:", K_y)
print("Gyro Z - B:", B_z, "at tau:", tau_B_z, "N:", N_z, "K:", K_z)
