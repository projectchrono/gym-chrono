from tensorflow.python.summary.summary_iterator import summary_iterator
import os
import matplotlib.pyplot as plt
import re
import sys

plt.rcParams.update({
    'axes.titlesize': 20,
    'axes.labelsize': 18,
    'lines.linewidth': 1.5,
    'lines.markersize': 6,
    'xtick.labelsize': 16,
    'ytick.labelsize': 16,
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'Palatino', 'serif'],
    # "font.serif" : ["Computer Modern Serif"],
})


def extract_data_from_event_file(event_file_path, tag):
    data = []
    for e in summary_iterator(event_file_path):
        for v in e.summary.value:
            if v.tag == tag:
                data.append((e.step, v.simple_value))
    return data


# Replace with your TensorBoard log directory and the tag you are interested in
log_dir = sys.argv[1]

tag1 = 'rollout/total_success'
tag2 = 'rollout/total_episode_num'
tag3 = 'rollout/total_crashes'
tag4 = 'rollout/total_timeout'

# Custom sorting function


def numeric_sort_key(s):
    return [int(text) if text.isdigit() else text.lower() for text in re.split('([0-9]+)', s)]


all_data_suc = []
all_data_epi = []
all_data_crash = []
all_data_timeout = []

# Walk through the base directory
for root, dirs, files in os.walk(log_dir, topdown=True):
    # Sort directories numerically
    dirs.sort(key=numeric_sort_key)
    for dir in sorted(dirs, key=numeric_sort_key):
        print(dir)
        dir_path = os.path.join(root, dir)
        for file in sorted(os.listdir(dir_path), key=numeric_sort_key):
            if file.startswith("events.out.tfevents"):
                event_file_path = os.path.join(dir_path, file)
                all_data_suc.extend(
                    extract_data_from_event_file(event_file_path, tag1))
                all_data_epi.extend(
                    extract_data_from_event_file(event_file_path, tag2))
                all_data_crash.extend(
                    extract_data_from_event_file(event_file_path, tag3))
                all_data_timeout.extend(
                    extract_data_from_event_file(event_file_path, tag4))


# Sorting the data by step
all_data_suc.sort(key=lambda x: x[0])
steps, suc = zip(*all_data_suc)

all_data_epi.sort(key=lambda x: x[0])
_, epi = zip(*all_data_epi)

all_data_crash.sort(key=lambda x: x[0])
_, crash = zip(*all_data_crash)

all_data_timeout.sort(key=lambda x: x[0])
_, timeout = zip(*all_data_timeout)

# convert epi and suc from tuple to list
epi = list(epi)
suc = list(suc)
crash = list(crash)
timeout = list(timeout)
steps = list(steps)

# Find the index where the number of episodes decreses compared to the previous one
index = []
for i in range(len(steps)):
    if i == 0:
        continue
    if epi[i] < epi[i-1]:
        index.append(i)

# Add these indexes, add the previous number to all the other elements
for i in index:
    for j in range(i, len(steps)):
        epi[j] += epi[i-1]

# At these indexs do same for suc
for i in index:
    for j in range(i, len(steps)):
        suc[j] += suc[i-1]

# Do the same for crash
for i in index:
    for j in range(i, len(steps)):
        crash[j] += crash[i-1]

# Do the same for timeout
for i in index:
    for j in range(i, len(steps)):
        timeout[j] += timeout[i-1]


# Plot of raw number of succes vs number of episodes
plt.figure(figsize=(15, 5))
plt.plot(epi, suc, linewidth=2.5, color='darkblue')
plt.xlabel('Number of Episodes run')
plt.ylabel('Number of Goal Reached')
plt.legend()
plt.xlim(0)
# Set x ticks every 500 episodes
# Rotate x tick labels by 45 degrees
if (log_dir == '../art_logs_3'):
    plt.xticks(range(0, int(epi[-1])+1, 10000), rotation=45, ha='right')
if (log_dir == '../art_logs_lidar_2'):
    plt.xticks(range(0, int(epi[-1])+1, 1000), rotation=45, ha='right')

plt.grid(True, linestyle=':', alpha=0.5)

# Set y lim max same as x lim
plt.ylim(0, int(epi[-1]))

# Draw line for 100% success rate (x = y line)
plt.plot(epi, epi, color='red', linestyle='--', label='100% Success Rate')


# Draw line for 75% success rate (y = 0.75x line)
plt.plot(epi, [0.75*x for x in epi], color='cyan',
         linestyle='--', label='75% Success Rate')


# Draw line for 50% success rate (y = 0.5x line)
plt.plot(epi, [0.5*x for x in epi], color='purple',
         linestyle='--', label='50% Success Rate')


# Set the line color to lighter shade
for line in plt.gca().lines:
    line.set_alpha(0.5)

# Add legend
plt.legend()
plt.tight_layout()
plt.show()

# Compute the success rate by taking 100 epsiode windows
# and computing the success rate in each window
success_rate = []
num_update = 1
for i in range(len(epi)):
    if i < num_update:
        continue
    success_rate.append((suc[i]-suc[i-num_update]) /
                        ((epi[i]-epi[i-num_update])))
# Do the same for crash
crash_rate = []
num_update = 1
for i in range(len(epi)):
    if i < num_update:
        continue
    crash_rate.append((crash[i]-crash[i-num_update]) /
                      ((epi[i]-epi[i-num_update])))
# Do the same for timeout
timeout_rate = []
num_update = 1
for i in range(len(epi)):
    if i < num_update:
        continue
    timeout_rate.append((timeout[i]-timeout[i-num_update]) /
                        ((epi[i]-epi[i-num_update])))


# Calculate moving average
window_size = 10
moving_average = []
for i in range(len(success_rate)):
    if i < window_size:
        moving_average.append(sum(success_rate[:i+1]) / (i+1))
    else:
        moving_average.append(
            sum(success_rate[i-window_size+1:i+1]) / window_size)

# Do the same for crash
window_size = 10
moving_average_crash = []
for i in range(len(crash_rate)):
    if i < window_size:
        moving_average_crash.append(sum(crash_rate[:i+1]) / (i+1))
    else:
        moving_average_crash.append(
            sum(crash_rate[i-window_size+1:i+1]) / window_size)

# Do the same for timeout
window_size = 10
moving_average_timeout = []
for i in range(len(timeout_rate)):
    if i < window_size:
        moving_average_timeout.append(sum(timeout_rate[:i+1]) / (i+1))
    else:
        moving_average_timeout.append(
            sum(timeout_rate[i-window_size+1:i+1]) / window_size)

# Create a list that increments by 2*num_check_point
# This will be used as x axis
x_axis = [i for i in range(len(success_rate))]
# Plot the success rate
plt.figure(figsize=(15, 5))
# plt.plot(x_axis, success_rate,
#          linewidth=2.5, color='darkblue', label='Success Rate', alpha=0.75)
plt.plot(x_axis, moving_average,
         linewidth=2.5, color='darkblue', linestyle='-', label='Success', alpha=1)
# plt.plot(x_axis, crash_rate,
#          linewidth=2.5, color='red', label='Crash Rate', alpha=0.75)
plt.plot(x_axis, moving_average_crash,
         linewidth=2.5, color='red', linestyle='-', label='Crash', alpha=1)
# plt.plot(x_axis, timeout_rate,
#          linewidth=2.5, color='purple', label='Timeout Rate', alpha=0.75)
plt.plot(x_axis, moving_average_timeout,
         linewidth=2.5, color='purple', linestyle='-', label='Timeout', alpha=1)


# Add legend with slightly bigger font size
plt.legend(fontsize='large')

plt.xlabel('Number of updates')
plt.ylabel('Rate')
plt.grid()
plt.xlim(0)


# Rest of the code...

# Add dotted vertical lines
vertical_lines = []
if (log_dir == '../art_logs_3'):
    vertical_lines = [25*2, 35*2, 60*2, 85*2, 90*2, 95*2, 110*2]
if (log_dir == '../art_logs_lidar_4'):
    vertical_lines = [10, 20, 60, 100]
    hilly_rigid_terrain_lines = [200]
    deformable_rigid_terrain_lines = [400, 450, 550]

for line in vertical_lines:
    plt.axvline(x=line, linestyle='--', color='blue', linewidth=2)

for line in hilly_rigid_terrain_lines:
    plt.axvline(x=line, linestyle='--', color='green', linewidth=2)

for line in deformable_rigid_terrain_lines:
    plt.axvline(x=line, linestyle='--', color='orange', linewidth=2)

# for line in vertical_lines:
#     plt.axvline(x=line, linestyle='--', color='blue', linewidth=2)

# for line in hilly_rigid_terrain_lines:
#     plt.axvline(x=line, linestyle='--', color='green', linewidth=2)

# for line in deformable_rigid_terrain_lines:
#     plt.axvline(x=line, linestyle='--', color='orange', linewidth=2)

# Add x labels incrementing by 10 and rotated to avoid overlapping
plt.xticks(range(0, len(x_axis)+50, 50), rotation=45, ha='right')

plt.tight_layout()
# Save high res vector fig
# plt.savefig('success_rate_full.pdf', format='pdf', dpi=1200)
plt.savefig('success_rate_full.png', format='png', dpi=600)

plt.show()
