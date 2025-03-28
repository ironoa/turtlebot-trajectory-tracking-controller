import sys
import rosbag
import matplotlib.pyplot as plt

# Read data from bag
bag = rosbag.Bag(sys.argv[1])

# State published by the simulator
vehicleState_time = []
vehicleState_x = []
vehicleState_y = []
vehicleState_theta = []
vehicleState_linearvelocity = []
vehicleState_angularvelocity = []

for topic, msg, t in bag.read_messages():
    if topic == "/state":
        vehicleState_time.append(msg.data[0])
        vehicleState_x.append(msg.data[1])
        vehicleState_y.append(msg.data[2])
        vehicleState_theta.append(msg.data[3])
        vehicleState_linearvelocity.append(msg.data[4])
        vehicleState_angularvelocity.append(msg.data[5])

bag.close()

# Plot data
plt.figure(1)
plt.plot(vehicleState_x,vehicleState_y)
plt.plot(vehicleState_x[0],vehicleState_y[0],'ro')
plt.plot(vehicleState_x[len(vehicleState_x)-1],vehicleState_y[len(vehicleState_x)-1],'rx')
plt.xlabel("x [m]")
plt.ylabel("y [m]")

plt.figure(2)
plt.subplot(211)
plt.plot(vehicleState_time,vehicleState_linearvelocity)
plt.xlabel("Time [s]")
plt.ylabel("Linear velocity [m/s]")
plt.subplot(212)
plt.plot(vehicleState_time,vehicleState_angularvelocity)
plt.xlabel("Time [s]")
plt.ylabel("Angular velocity [rad/s]")

plt.figure(3)
plt.subplot(311)
plt.plot(vehicleState_time,vehicleState_x)
plt.xlabel("Time [s]")
plt.ylabel("x [m]")
plt.subplot(312)
plt.plot(vehicleState_time,vehicleState_y)
plt.xlabel("Time [s]")
plt.ylabel("y [m]")
plt.subplot(313)
plt.plot(vehicleState_time,vehicleState_theta)
plt.xlabel("Time [s]")
plt.ylabel("theta [rad]")

plt.show()

