import pandas as pd
import matplotlib.pyplot as plt

# Replace with the actual path to your CSV file
csv_file_path = 'motor_data_turn_50pulses_kp0.2_ki0.08_kd0.1.csv'

# Read the CSV file into a DataFrame
df = pd.read_csv(csv_file_path)

print(df.columns)

# Extract the data columns
time = df.index  # Assuming the index represents time
encoder1 = df['Encoder 1']
encoder2 = df['Encoder 2']
control_signal1 = df['Control Signal 1']
control_signal2 = df['Control Signal 2']
pid_speed_control = df['PID Speed Control']
motor_speed1 = df['Motor Speed 1']
motor_speed2 = df['Motor Speed 2']

# Create subplots for each variable
fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

# Plot Encoder 1 and 2 in a single graph
axs[0].plot(time, encoder1, label='Encoder 1', color='red')
axs[0].plot(time, encoder2, label='Encoder 2', color='blue')
axs[0].set_ylabel('Encoder Counts')
axs[0].legend()

# Plot Control Signal 1 and 2 in one graph
axs[1].plot(time, control_signal1, label='Control Signal 1', color='red')
axs[1].plot(time, control_signal2, label='Control Signal 2', color='blue')
axs[1].set_ylabel('Control Signal')
axs[1].legend()

# Plot PID Speed Control
axs[2].plot(time, pid_speed_control, label='PID Speed Control', color='green')
axs[2].set_ylabel('PID Speed Control')
axs[2].legend()

# Plot Motor Speed 1 and 2 in a single graph
axs[3].plot(time, motor_speed1, label='Motor Speed 1 (Red)', color='red')
axs[3].plot(time, motor_speed2, label='Motor Speed 2 (Blue)', color='blue')
axs[3].set_ylabel('Motor Speed')
axs[3].set_xlabel('Time')
axs[3].legend()

# Display the plots
plt.tight_layout()
plt.show()
