

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Set up the figure and axis
fig, ax = plt.subplots()
line, = ax.plot([], [], label='Sensor Data')
ax.set_title('Real-time Data Plot')
ax.set_xlabel('Time')
ax.set_ylabel('Sensor Value')
ax.legend()

# Initialize empty lists to store data
times = []
values = []

# Function to update the plot in real-time
def update_plot(frame):
    if ser.in_waiting > 0:
        data = ser.readline().decode('latin-1').strip()
        if data:
            try:
                value = float(data)
                times.append(frame)
                values.append(value)
                line.set_data(times, values)
                ax.relim()
                ax.autoscale_view()
            except ValueError:
                print("Invalid data:", data)

# Set up the serial connection
ser = serial.Serial('COM5', 9600)  # Replace 'COM5' with the appropriate port name
if not ser.isOpen():
    ser.open()

# Set up the animation
ani = FuncAnimation(fig, update_plot, blit=False)

try:
    plt.show()
except KeyboardInterrupt:
    ser.close()
