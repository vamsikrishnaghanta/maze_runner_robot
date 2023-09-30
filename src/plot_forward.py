import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Replace with the actual path to your CSV file
csv_file_path = 'motor_data_forward_1000pulses_kp0.5_ki0.1_kd0.5.csv'

# Read the CSV file into a DataFrame
df = pd.read_csv(csv_file_path)

# Extract the data columns
time = df.index  # Assuming the index represents time
right_encoder1 = df['Right Encoder 1']
right_encoder2 = df['Right Encoder 2']
left_encoder1 = df['Left Encoder 1']
left_encoder2 = df['Left Encoder 2']
pid_speed_control = df['PID Speed Control']
speed_difference = df['Speed Difference']
motor_speed1 = df['Motor Speed 1']
motor_speed2 = df['Motor Speed 2']
# Create subplots for each variable
fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

# Plot Encoder 1 and 2 in a single graph
axs[0].plot(time, right_encoder1, label='Right Encoder 1', color='red')
axs[0].plot(time, right_encoder2, label='Right Encoder 2', color='red', linestyle='--')
axs[0].plot(time, left_encoder1, label='Left Encoder 1', color='blue')
axs[0].plot(time, left_encoder2, label='Left Encoder 2', color='blue', linestyle='--')
axs[0].set_ylabel('Encoder Counts')
axs[0].legend()

# Plot Motor Speed 1 and 2 in a single graph
axs[2].plot(time, motor_speed1, label='Motor Speed 1 (Red)', color='red')
axs[2].plot(time, motor_speed2, label='Motor Speed 2 (Blue)', color='blue')
axs[2].set_ylabel('Motor Speed')
axs[2].set_xlabel('Time')
axs[2].legend()

# Create a figure and axis
ax1 = axs[1]

# Plot the first dataset on the left y-axis
color = 'tab:green'
ax1.plot(time, speed_difference, label='Speed Difference', color=color)
ax1.set_ylabel('Speed Difference', color=color)
ax1.tick_params(axis='y', labelcolor=color)

# Create a second y-axis on the right side
ax2 = ax1.twinx()

# Plot the second dataset on the right y-axis
color = 'tab:red'
ax2.plot(time, pid_speed_control, label='PID Speed Control', color=color)
ax2.set_ylabel('PID Speed Control', color=color)
ax2.tick_params(axis='y', labelcolor=color)

# Set common x-axis label and legend
ax1.set_xlabel('Time')
axs[1].legend()
ax2.legend(loc='lower left')


# Display the plots
plt.tight_layout()
plt.show()
