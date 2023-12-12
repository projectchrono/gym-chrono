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

tag = 'rollout/ep_rew_mean'


# Custom sorting function
def numeric_sort_key(s):
    return [int(text) if text.isdigit() else text.lower() for text in re.split('([0-9]+)', s)]


all_data = []
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
                all_data.extend(
                    extract_data_from_event_file(event_file_path, tag))


# Sorting the data by step
all_data.sort(key=lambda x: x[0])
steps, values = zip(*all_data)

index = list(range(len(steps)))

plt.figure(figsize=(15, 5))
plt.plot(index, values)
plt.xlabel('Update')
plt.ylabel('Reward')
plt.title(f'Training Progress for {tag}')
plt.legend()
plt.xticks(range(0, len(steps)+10, 10), rotation=45, ha='right')
plt.xlim(0)
plt.grid(True, linestyle=':', alpha=0.5)

# Add dotted vertical lines
if (log_dir == '../art_logs_3'):
    vertical_lines = [25*2, 35*2, 60*2, 85*2, 90*2, 95*2, 110*2]
if (log_dir == '../art_logs_lidar_2'):
    vertical_lines = [25*2, 40*2, 60*2, 65 *
                      2, 70*2, 75*2, 80*2, 85*2, 90*2, 95*2]
for line in vertical_lines:
    plt.axvline(x=line, linestyle=':', color='black', linewidth=1)

plt.tight_layout()
plt.show()
