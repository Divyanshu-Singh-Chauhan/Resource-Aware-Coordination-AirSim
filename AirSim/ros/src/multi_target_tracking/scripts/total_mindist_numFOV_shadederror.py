import os
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# SELECT WHICH EVADER MOTION SET TO PLOT
# evader_motion_to_plot = "sto"
# evader_motion_to_plot = "lin"
evader_motion_to_plot = "herd"
algorithm_to_plot = "RBG"

# Set working directory
# dir_path = os.getcwd()
dir_path = "/home/sgari/sandilya_ws/unity_airsim_workspace/experiments_data_mtt/"

# Get list of CSV files
files = [f for f in os.listdir(dir_path) if f.startswith(
    'output') and f.endswith('.csv')]

# Initialize variables
unique_freqs = []
fig_indices = 0
unique_data = {}
time_data = {}
num_fov_multi_trials = {}

for i, file in enumerate(files):

    # Get frequency
    parts = file.split('_')
    freq = float(parts[3])
    evader_motion = parts[-1].split(".")[0]

    alg = parts[2]
    if not (alg == algorithm_to_plot):
        continue

    if not (evader_motion == evader_motion_to_plot):
        continue 

    if freq not in unique_freqs:
        # New frequency
        fig_indices += 1
        unique_freqs.append(freq)

        # Initialize
        unique_data[freq] = []
        num_fov_multi_trials[freq] = []
        # time_data[freq] = []

    # Read CSV
    with open(os.path.join(dir_path, file), 'r') as f:
        reader = csv.reader(f)
        # data = np.array([row for row in reader]).astype(float)
        data = np.array([row for row in reader])

    # Extract time and min dist
    if np.size(data) == 0:
        print(file, " is empty!!!")
        continue

    time_data_s = data[:, 0]
    min_dist_data = data[:, 1]
    total_num_evaders_inFOV = data[:, 2]

    # Store time data
    # freq_str = str(freq)
    if freq not in time_data:
        time_data[freq] = time_data_s

    # Store min dist data
    unique_data[freq].append(min_dist_data)
    num_fov_multi_trials[freq].append(total_num_evaders_inFOV)

# Trim columns to shortest
for freq in unique_data:
    min_len = min([len(arr) for arr in unique_data[freq]])
    for i in range(len(unique_data[freq])):
        unique_data[freq][i] = unique_data[freq][i][:min_len]
        num_fov_multi_trials[freq][i] = num_fov_multi_trials[freq][i][:min_len]
    time_data[freq] = time_data[freq][:min_len]

# Plot each frequency
# for fig_idx in range(len(unique_data)):

for fig_idx in unique_data:
  # Calculate stats
  # Concatenate arrays horizontally
    plt.figure()
    concat_data = np.vstack(unique_data[fig_idx])
    concat_data = concat_data.transpose().astype(np.float)
    mean_data = np.mean(concat_data, axis=1)
    std_data = np.std(concat_data, axis=1)

    # Plot
    # plt.errorbar(time_data[fig_idx], mean_data, yerr=std_data, fmt='-k')
    plt.errorbar(time_data[fig_idx].astype(np.float), mean_data, yerr=std_data, ecolor='c', fmt='none')
    plt.plot(time_data[fig_idx].astype(np.float), mean_data, '-', color='orange', linewidth=2)

    # Set x-axis ticks in 10-second intervals
    # Get min and max time
    tmin = time_data[fig_idx][0].astype(np.float)
    tmax = time_data[fig_idx][-1].astype(np.float)

    # Calculate tick interval to always give ~10 ticks
    tick_interval = (tmax - tmin) / 10

    # Generate ticks
    ticks = np.arange(tmin, tmax, tick_interval)

    # Round tick labels to 1 decimal 
    ticks_rounded = np.round(ticks, 1)

    # Set ticks
    plt.xticks(ticks, ticks_rounded)
    plt.gca().xaxis.set_major_formatter(ticker.FormatStrFormatter('%0.1f'))

    freq = fig_idx
    plt.title(f'{algorithm_to_plot} total min distance with replanning frequency = {freq} Hz')
    plt.xlabel('Time')
    plt.ylabel('Mean Total Min Distance with Standard Deviation')
    # plt.legend()
    # plt.grid()

# plt.show()



for fig_idx in num_fov_multi_trials:
  # Calculate stats
  # Concatenate arrays horizontally
    plt.figure()
    concat_data = np.vstack(num_fov_multi_trials[fig_idx])
    concat_data = concat_data.transpose().astype(np.float)
    mean_data = np.mean(concat_data, axis=1)
    std_data = np.std(concat_data, axis=1)

    # Plot
    # plt.errorbar(time_data[fig_idx], mean_data, yerr=std_data, fmt='-k')
    plt.errorbar(time_data[fig_idx].astype(np.float), mean_data, yerr=std_data, ecolor='c', fmt='none')
    plt.plot(time_data[fig_idx].astype(np.float), mean_data, '-', color='orange', linewidth=2)

    # Set x-axis ticks in 10-second intervals
    # Get min and max time
    tmin = time_data[fig_idx][0].astype(np.float)
    tmax = time_data[fig_idx][-1].astype(np.float)

    # Calculate tick interval to always give ~10 ticks
    tick_interval = (tmax - tmin) / 10

    # Generate ticks
    ticks = np.arange(tmin, tmax, tick_interval)

    # Round tick labels to 1 decimal 
    ticks_rounded = np.round(ticks, 1)

    # Set ticks
    plt.xticks(ticks, ticks_rounded)
    plt.gca().xaxis.set_major_formatter(ticker.FormatStrFormatter('%0.1f'))

    freq = fig_idx
    plt.title(f'{algorithm_to_plot} algorithm evaders in FOV with replanning frequency = {freq} Hz')
    plt.xlabel('Time')
    plt.ylabel('Mean total number of evaders in FOV with std. dev.')
    # plt.legend()
    # plt.grid()

plt.show()