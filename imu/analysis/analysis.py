import rosbag
import numpy as np
import matplotlib.pyplot as plt

def read_rosbag(bag_file, topic='/imu'):
    bag = rosbag.Bag(bag_file)
    data = {'orientation_x':[], 'orientation_y':[], 'orientation_z':[], 'orientation_w':[], 'magnetic_field_x': [], 'magnetic_field_y': [], 'magnetic_field_z': [], 'linear_acceleration_x': [], 'linear_acceleration_y': [], 'linear_acceleration_z': [], 'angular_velocity_x': [], 'angular_velocity_y': [], 'angular_velocity_z': [], 'time':[]}

    for topic, msg, t in bag.read_messages(topics=[topic]):

        data['orientation_x'].append(msg.imu.orientation.x)
        data['orientation_y'].append(msg.imu.orientation.y)
        data['orientation_z'].append(msg.imu.orientation.z)
        data['orientation_w'].append(msg.imu.orientation.w)

        data['magnetic_field_x'].append(msg.mag_field.magnetic_field.x)
        data['magnetic_field_y'].append(msg.mag_field.magnetic_field.y)
        data['magnetic_field_z'].append(msg.mag_field.magnetic_field.z)

        data['linear_acceleration_x'].append(msg.imu.linear_acceleration.x)
        data['linear_acceleration_y'].append(msg.imu.linear_acceleration.y)
        data['linear_acceleration_z'].append(msg.imu.linear_acceleration.z)

        data['angular_velocity_x'].append(msg.imu.angular_velocity.x)
        data['angular_velocity_y'].append(msg.imu.angular_velocity.y)
        data['angular_velocity_z'].append(msg.imu.angular_velocity.z)

        data['time'].append(msg.header.stamp.to_sec())

    bag.close()

    for key in data:
        data[key] = np.array(data[key])

    return data

def to_euler_angles(qx,qy,qz,qw):
    yaw = np.arctan2(2 * (qx*qy + qw*qz), qw**2 + qx**2 - qy**2 - qz**2)
    pitch = np.arcsin(2 * (qw*qy - qx*qz))
    roll = np.arctan2(2 * (qw*qx + qy*qz), qw**2 - qx**2 - qy**2 + qz**2)
    
    return yaw, pitch, roll

def plot_histogram(ax, data, bins, color, alpha, label, xlabel, ylabel, title):
    ax.hist(data, bins=bins, color=color, alpha=alpha, label=label)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    ax.grid(True)

data_imu=read_rosbag("/home/shivakumardhandapani/imu/data/output.bag")

plt.figure(figsize=(8,6))
plt.plot(data_imu['time'], data_imu['linear_acceleration_x'], color='red', label='Linear acceleration X', marker='o')
plt.plot(data_imu['time'], data_imu['linear_acceleration_y'], color='blue', label='Linear acceleration Y', marker='*')
plt.plot(data_imu['time'], data_imu['linear_acceleration_z'], color='green', label='Linear acceleration Z', marker='x')
plt.title("Acceleration (m/s^2)")
plt.xlabel("Time (sec)")
plt.ylabel("Linear Acceleration (m/s^2)")
plt.grid(True)
plt.legend()
plt.show()

plt.figure(figsize=(8,6))
plt.plot(data_imu['time'], data_imu['angular_velocity_x'], color='red', label='Angular Velocity X', marker='o')
plt.plot(data_imu['time'], data_imu['angular_velocity_y'], color='blue', label='Angular Velocity Y', marker='*')
plt.plot(data_imu['time'], data_imu['angular_velocity_z'], color='green', label='Angular Velocity Z', marker='x')
plt.title("Rotational rate (degrees/s)")
plt.xlabel("Time (sec)")
plt.ylabel("Angular Velocity (degrees/s)")
plt.grid(True)
plt.legend()
plt.show()

yaw,pitch,roll=to_euler_angles(data_imu['orientation_x'],data_imu['orientation_y'],data_imu['orientation_z'],data_imu['orientation_w'])

plt.figure(figsize=(8,6))
plt.plot(data_imu['time'], yaw, color='red', label='Yaw', marker='o')
plt.plot(data_imu['time'], pitch, color='blue', label='Pitch', marker='*')
plt.plot(data_imu['time'], roll, color='green', label='Roll', marker='x')
plt.title("Rotation from the VN estimation (degress)")
plt.xlabel("Time (sec)")
plt.ylabel("Rotation (degrees)")
plt.grid(True)
plt.legend()
plt.show()

plt.figure(figsize=(24,6))
plt.subplot(1,3,1)
plot_histogram(plt.gca(), yaw, bins=30, color='red', alpha=0.5, label='Yaw', xlabel='Yaw', ylabel='Frequency', title='Yaw Rotation 1D histogram')
plt.grid(True)
plt.subplot(1,3,2)
plot_histogram(plt.gca(), pitch, bins=30, color='blue', alpha=0.5, label='Pitch', xlabel='Pitch', ylabel='Frequency', title='Pitch Rotation 1D histogram')
plt.grid(True)
plt.subplot(1,3,3)
plot_histogram(plt.gca(), roll, bins=30, color='green', alpha=0.5, label='Roll', xlabel='Roll', ylabel='Frequency', title='Roll Rotation 1D histogram')
plt.grid(True)
plt.show()